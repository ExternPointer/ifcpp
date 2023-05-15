#pragma once


namespace ifcpp {

class Parameters {
public:
    double m_epsilon;
    int m_numVerticesPerCircle;
    int m_minNumVerticesPerArc;
    double m_modelMaxSize;
    int m_numVerticesPerControlPoint;
    double m_lengthFactor;
    double m_angleFactor;
};

}