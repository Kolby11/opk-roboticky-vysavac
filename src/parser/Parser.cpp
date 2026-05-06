#include "parser/Parser.h"

#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
  struct ParsedLine
  {
    int indent{0};
    bool is_list_item{false};
    std::string content;
  };

  std::string trim(const std::string &value)
  {
    std::size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])))
      ++start;

    std::size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])))
      --end;

    return value.substr(start, end - start);
  }

  std::string strip_comment(const std::string &line)
  {
    bool in_single_quotes = false;
    bool in_double_quotes = false;

    for (std::size_t i = 0; i < line.size(); ++i)
    {
      const char c = line[i];
      if (c == '\'' && !in_double_quotes)
        in_single_quotes = !in_single_quotes;
      else if (c == '"' && !in_single_quotes)
        in_double_quotes = !in_double_quotes;
      else if (c == '#' && !in_single_quotes && !in_double_quotes)
        return line.substr(0, i);
    }

    return line;
  }

  ParsedLine parse_line(const std::string &raw_line)
  {
    ParsedLine line;
    while (line.indent < static_cast<int>(raw_line.size()) && raw_line[line.indent] == ' ')
      ++line.indent;

    line.content = trim(strip_comment(raw_line.substr(line.indent)));
    if (!line.content.empty() && line.content.front() == '-')
    {
      line.is_list_item = true;
      line.content = trim(line.content.substr(1));
    }

    return line;
  }

  std::string parse_scalar(const std::string &value)
  {
    std::string parsed = trim(value);
    if (parsed.size() >= 2)
    {
      const char first = parsed.front();
      const char last = parsed.back();
      if ((first == '"' && last == '"') || (first == '\'' && last == '\''))
        return parsed.substr(1, parsed.size() - 2);
    }
    return parsed;
  }

  std::size_t find_separator(const std::string &content)
  {
    bool in_single_quotes = false;
    bool in_double_quotes = false;

    for (std::size_t i = 0; i < content.size(); ++i)
    {
      const char c = content[i];
      if (c == '\'' && !in_double_quotes)
        in_single_quotes = !in_single_quotes;
      else if (c == '"' && !in_single_quotes)
        in_double_quotes = !in_double_quotes;
      else if (c == ':' && !in_single_quotes && !in_double_quotes)
        return i;
    }

    return std::string::npos;
  }

  std::pair<std::string, std::string> parse_key_value(const std::string &content)
  {
    const std::size_t separator = find_separator(content);
    if (separator == std::string::npos)
      throw std::runtime_error("Expected 'key: value', got: " + content);

    return {
        trim(content.substr(0, separator)),
        trim(content.substr(separator + 1))};
  }

  struct ParserState
  {
    const std::vector<ParsedLine> &lines;
    std::size_t index{0};

    YAMLValue parse_block(int expected_indent)
    {
      if (index >= lines.size())
        return YAMLValue::makeMap();

      if (lines[index].indent < expected_indent)
        throw std::runtime_error("Invalid indentation");

      if (lines[index].indent > expected_indent)
        throw std::runtime_error("Unexpected indentation");

      if (lines[index].is_list_item)
        return parse_sequence(expected_indent);

      return parse_map(expected_indent);
    }

    YAMLValue parse_map(int expected_indent)
    {
      YAMLValue result = YAMLValue::makeMap();

      while (index < lines.size())
      {
        const ParsedLine &line = lines[index];
        if (line.indent < expected_indent)
          break;
        if (line.indent > expected_indent)
          throw std::runtime_error("Unexpected indentation in map");
        if (line.is_list_item)
          throw std::runtime_error("Cannot mix list item with map entry at same indentation");

        const auto [key, value] = parse_key_value(line.content);
        ++index;

        if (value.empty())
        {
          if (index < lines.size() && lines[index].indent > expected_indent)
            result.map.emplace(key, parse_block(lines[index].indent));
          else
            result.map.emplace(key, YAMLValue::makeScalar(""));
        }
        else
        {
          result.map.emplace(key, YAMLValue::makeScalar(parse_scalar(value)));
        }
      }

      return result;
    }

    YAMLValue parse_sequence(int expected_indent)
    {
      YAMLValue result = YAMLValue::makeSequence();

      while (index < lines.size())
      {
        const ParsedLine &line = lines[index];
        if (line.indent < expected_indent)
          break;
        if (line.indent > expected_indent)
          throw std::runtime_error("Unexpected indentation in sequence");
        if (!line.is_list_item)
          break;

        ++index;

        if (line.content.empty())
        {
          if (index < lines.size() && lines[index].indent > expected_indent)
            result.sequence.push_back(parse_block(lines[index].indent));
          else
            result.sequence.push_back(YAMLValue::makeScalar(""));
          continue;
        }

        const std::size_t separator = find_separator(line.content);
        if (separator == std::string::npos)
        {
          result.sequence.push_back(YAMLValue::makeScalar(parse_scalar(line.content)));
          continue;
        }

        YAMLValue item = YAMLValue::makeMap();
        const auto [key, value] = parse_key_value(line.content);
        if (value.empty())
        {
          if (index < lines.size() && lines[index].indent > expected_indent)
            item.map.emplace(key, parse_block(lines[index].indent));
          else
            item.map.emplace(key, YAMLValue::makeScalar(""));
        }
        else
        {
          item.map.emplace(key, YAMLValue::makeScalar(parse_scalar(value)));
        }

        while (index < lines.size() && lines[index].indent > expected_indent)
        {
          const ParsedLine &child_line = lines[index];
          if (child_line.is_list_item)
            throw std::runtime_error("Nested list items require an explicit key");

          const auto [child_key, child_value] = parse_key_value(child_line.content);
          if (child_line.indent == expected_indent + 2)
          {
            ++index;
            if (child_value.empty())
            {
              if (index < lines.size() && lines[index].indent > child_line.indent)
                item.map.emplace(child_key, parse_block(lines[index].indent));
              else
                item.map.emplace(child_key, YAMLValue::makeScalar(""));
            }
            else
            {
              item.map.emplace(child_key, YAMLValue::makeScalar(parse_scalar(child_value)));
            }
            continue;
          }

          throw std::runtime_error("Invalid indentation inside sequence item");
        }

        result.sequence.push_back(item);
      }

      return result;
    }
  };
} // namespace

YAMLValue YAMLValue::makeScalar(const std::string &value)
{
  YAMLValue node;
  node.type = Type::Scalar;
  node.scalar = value;
  return node;
}

YAMLValue YAMLValue::makeMap()
{
  YAMLValue node;
  node.type = Type::Map;
  return node;
}

YAMLValue YAMLValue::makeSequence()
{
  YAMLValue node;
  node.type = Type::Sequence;
  return node;
}

bool YAMLValue::isScalar() const
{
  return type == Type::Scalar;
}

bool YAMLValue::isMap() const
{
  return type == Type::Map;
}

bool YAMLValue::isSequence() const
{
  return type == Type::Sequence;
}

YAMLDocument YAMLParser::parseString(const std::string &content)
{
  std::vector<ParsedLine> lines;
  std::string current_line;

  for (char c : content)
  {
    if (c == '\r')
      continue;

    if (c == '\n')
    {
      ParsedLine parsed = parse_line(current_line);
      if (!parsed.content.empty())
        lines.push_back(parsed);
      current_line.clear();
      continue;
    }

    current_line.push_back(c);
  }

  ParsedLine parsed = parse_line(current_line);
  if (!parsed.content.empty())
    lines.push_back(parsed);

  YAMLDocument document;
  if (lines.empty())
    return document;

  ParserState state{lines};
  document.root = state.parse_block(lines.front().indent);
  if (state.index != lines.size())
    throw std::runtime_error("Failed to consume complete YAML input");

  return document;
}

YAMLDocument YAMLParser::parseFile(const std::string &filename)
{
  std::ifstream input(filename);
  if (!input)
    throw std::runtime_error("Could not open YAML file: " + filename);

  std::string content((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  return parseString(content);
}
