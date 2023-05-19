/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcMaterialProfileSetUsage.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcMaterialProfileSet;
	class IFCQUERY_EXPORT IfcCardinalPointReference;
	//ENTITY
	class IFCQUERY_EXPORT IfcMaterialProfileSetUsageTapering : public IfcMaterialProfileSetUsage
	{
	public:
		IfcMaterialProfileSetUsageTapering() = default;
		IfcMaterialProfileSetUsageTapering( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 5; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 3404854881; }

		// IfcMaterialUsageDefinition -----------------------------------------------------------
		// inverse attributes:
		//  std::vector<weak_ptr<IfcRelAssociatesMaterial> >	m_AssociatedTo_inverse;

		// IfcMaterialProfileSetUsage -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcMaterialProfileSet>					m_ForProfileSet;
		//  shared_ptr<IfcCardinalPointReference>				m_CardinalPoint;			//optional
		//  shared_ptr<IfcPositiveLengthMeasure>				m_ReferenceExtent;			//optional

		// IfcMaterialProfileSetUsageTapering -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcMaterialProfileSet>					m_ForProfileEndSet;
		shared_ptr<IfcCardinalPointReference>				m_CardinalEndPoint;			//optional
	};
}
