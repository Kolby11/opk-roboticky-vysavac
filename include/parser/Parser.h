#pragma once

#include <map>
#include <string>
#include <vector>

struct YAMLValue
{
    enum class Type
    {
        Scalar,
        Map,
        Sequence
    };

    Type type{Type::Scalar};
    std::string scalar;
    std::map<std::string, YAMLValue> map;
    std::vector<YAMLValue> sequence;

    static YAMLValue makeScalar(const std::string &value);
    static YAMLValue makeMap();
    static YAMLValue makeSequence();

    bool isScalar() const;
    bool isMap() const;
    bool isSequence() const;
};

struct YAMLDocument
{
    YAMLValue root{YAMLValue::makeMap()};
};

class YAMLParser
{
public:
    static YAMLDocument parseString(const std::string &content);
    static YAMLDocument parseFile(const std::string &filename);
};
