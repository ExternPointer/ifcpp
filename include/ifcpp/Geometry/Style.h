#pragma once


namespace ifcpp {

class Style {
public:
    enum Type {
        CURVE,
        SURFACE_FRONT,
        SURFACE_BACK,
        SURFACE_BOTH,
        UNDEFINED,
    };
    class Color {
    public:
        float r = 0, g = 0, b = 0, a = 0;
    };
    Color m_color;
    Type m_type = UNDEFINED;
};

}