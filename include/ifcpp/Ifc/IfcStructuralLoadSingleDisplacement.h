/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcStructuralLoadStatic.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcLengthMeasure;
	class IFCQUERY_EXPORT IfcPlaneAngleMeasure;
	//ENTITY
	class IFCQUERY_EXPORT IfcStructuralLoadSingleDisplacement : public IfcStructuralLoadStatic
	{
	public:
		IfcStructuralLoadSingleDisplacement() = default;
		IfcStructuralLoadSingleDisplacement( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 7; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 2473145415; }

		// IfcStructuralLoad -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcLabel>				m_Name;						//optional

		// IfcStructuralLoadOrResult -----------------------------------------------------------

		// IfcStructuralLoadStatic -----------------------------------------------------------

		// IfcStructuralLoadSingleDisplacement -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcLengthMeasure>		m_DisplacementX;			//optional
		shared_ptr<IfcLengthMeasure>		m_DisplacementY;			//optional
		shared_ptr<IfcLengthMeasure>		m_DisplacementZ;			//optional
		shared_ptr<IfcPlaneAngleMeasure>	m_RotationalDisplacementRX;	//optional
		shared_ptr<IfcPlaneAngleMeasure>	m_RotationalDisplacementRY;	//optional
		shared_ptr<IfcPlaneAngleMeasure>	m_RotationalDisplacementRZ;	//optional
	};
}
