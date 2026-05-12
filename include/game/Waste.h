#pragma once

#include <memory>
#include <string>

#include "environment/Environment.h"
#include "types/Geometry.h"

namespace game
{
    class Waste
    {
    public:
        Waste(std::string type, std::string color, geometry::Point2d position, double radius);
        virtual ~Waste() = default;

        virtual std::unique_ptr<Waste> clone() const = 0;

        const std::string &getType() const;
        const std::string &getColor() const;
        const geometry::Point2d &getPosition() const;
        double getRadius() const;

    private:
        std::string type_;
        std::string color_;
        geometry::Point2d position_;
        double radius_;
    };

    class PaperWaste final : public Waste
    {
    public:
        PaperWaste(const std::string &color, geometry::Point2d position, double radius);
        std::unique_ptr<Waste> clone() const override;
    };

    class PlasticWaste final : public Waste
    {
    public:
        PlasticWaste(const std::string &color, geometry::Point2d position, double radius);
        std::unique_ptr<Waste> clone() const override;
    };

    class GlassWaste final : public Waste
    {
    public:
        GlassWaste(const std::string &color, geometry::Point2d position, double radius);
        std::unique_ptr<Waste> clone() const override;
    };

    class ConfiguredWaste final : public Waste
    {
    public:
        ConfiguredWaste(const std::string &type, const std::string &color, geometry::Point2d position, double radius);
        std::unique_ptr<Waste> clone() const override;
    };

    class WasteFactory
    {
    public:
        static std::unique_ptr<Waste> create(const environment::WasteType &type,
                                             geometry::Point2d position,
                                             double radius);
    };
} // namespace game
