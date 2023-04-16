#pragma once


namespace ifcpp {

class Parameters {
public:
    float m_lengthFactor;
    float m_angleFactor;
    float m_epsilon;
    int	m_NumVerticesPerCircle;
    int m_minNumVerticesPerArc;
    float m_modelMaxSize;
    int m_numVerticesPerControlPoint;
};

}