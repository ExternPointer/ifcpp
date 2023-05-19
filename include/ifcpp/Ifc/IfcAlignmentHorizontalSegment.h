/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcAlignmentParameterSegment.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcCartesianPoint;
	class IFCQUERY_EXPORT IfcPlaneAngleMeasure;
	class IFCQUERY_EXPORT IfcLengthMeasure;
	class IFCQUERY_EXPORT IfcNonNegativeLengthMeasure;
	class IFCQUERY_EXPORT IfcPositiveLengthMeasure;
	class IFCQUERY_EXPORT IfcAlignmentHorizontalSegmentTypeEnum;
	//ENTITY
	class IFCQUERY_EXPORT IfcAlignmentHorizontalSegment : public IfcAlignmentParameterSegment
	{
	public:
		IfcAlignmentHorizontalSegment() = default;
		IfcAlignmentHorizontalSegment( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 9; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 536804194; }

		// IfcAlignmentParameterSegment -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcLabel>								m_StartTag;					//optional
		//  shared_ptr<IfcLabel>								m_EndTag;					//optional

		// IfcAlignmentHorizontalSegment -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcCartesianPoint>						m_StartPoint;
		shared_ptr<IfcPlaneAngleMeasure>					m_StartDirection;
		shared_ptr<IfcLengthMeasure>						m_StartRadiusOfCurvature;
		shared_ptr<IfcLengthMeasure>						m_EndRadiusOfCurvature;
		shared_ptr<IfcNonNegativeLengthMeasure>				m_SegmentLength;
		shared_ptr<IfcPositiveLengthMeasure>				m_GravityCenterLineHeight;	//optional
		shared_ptr<IfcAlignmentHorizontalSegmentTypeEnum>	m_PredefinedType;
	};
}
