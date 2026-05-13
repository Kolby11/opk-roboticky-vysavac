#include "game/Waste.h"

#include <algorithm>
#include <cctype>
#include <utility>

namespace game
{
    namespace
    {
        std::string normalized(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(),
                           [](unsigned char c)
                           { return static_cast<char>(std::tolower(c)); });
            return value;
        }
    } // namespace

    Waste::Waste(std::string type, std::string color, geometry::Point2d position, double radius)
        : type_(std::move(type)),
          color_(std::move(color)),
          position_(position),
          radius_(radius)
    {
    }

    const std::string &Waste::getType() const { return type_; }
    const std::string &Waste::getColor() const { return color_; }
    const geometry::Point2d &Waste::getPosition() const { return position_; }
    double Waste::getRadius() const { return radius_; }
    CircleCollider Waste::getCollider() const { return CircleCollider(position_, radius_); }

    PaperWaste::PaperWaste(const std::string &color, geometry::Point2d position, double radius)
        : Waste("papier", color, position, radius)
    {
    }

    std::unique_ptr<Waste> PaperWaste::clone() const
    {
        return std::make_unique<PaperWaste>(getColor(), getPosition(), getRadius());
    }

    PlasticWaste::PlasticWaste(const std::string &color, geometry::Point2d position, double radius)
        : Waste("plast", color, position, radius)
    {
    }

    std::unique_ptr<Waste> PlasticWaste::clone() const
    {
        return std::make_unique<PlasticWaste>(getColor(), getPosition(), getRadius());
    }

    GlassWaste::GlassWaste(const std::string &color, geometry::Point2d position, double radius)
        : Waste("sklo", color, position, radius)
    {
    }

    std::unique_ptr<Waste> GlassWaste::clone() const
    {
        return std::make_unique<GlassWaste>(getColor(), getPosition(), getRadius());
    }

    ConfiguredWaste::ConfiguredWaste(const std::string &type, const std::string &color, geometry::Point2d position, double radius)
        : Waste(type, color, position, radius)
    {
    }

    std::unique_ptr<Waste> ConfiguredWaste::clone() const
    {
        return std::make_unique<ConfiguredWaste>(getType(), getColor(), getPosition(), getRadius());
    }

    std::unique_ptr<Waste> WasteFactory::create(const environment::WasteType &type,
                                                geometry::Point2d position,
                                                double radius)
    {
        const std::string name = normalized(type.name);

        if (name == "papier" || name == "paper")
            return std::make_unique<PaperWaste>(type.color, position, radius);

        if (name == "plast" || name == "plastic")
            return std::make_unique<PlasticWaste>(type.color, position, radius);

        if (name == "sklo" || name == "glass")
            return std::make_unique<GlassWaste>(type.color, position, radius);

        return std::make_unique<ConfiguredWaste>(type.name, type.color, position, radius);
    }
} // namespace game
