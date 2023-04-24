#pragma once


namespace ifcpp {

class Parameters {
public:
    float m_epsilon;
    int m_numVerticesPerCircle;
    int m_minNumVerticesPerArc;
    float m_modelMaxSize;
    int m_numVerticesPerControlPoint;
    float m_lengthFactor;
    float m_angleFactor;
};

}