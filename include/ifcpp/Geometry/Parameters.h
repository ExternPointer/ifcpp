#pragma once


namespace ifcpp {

class Parameters {
public:
    float m_lengthFactor;
    float m_angleFactor;
    float m_epsilon;
    int	m_NumVerticesPerCircle = 14;
    int m_minNumVerticesPerArc;
};

}