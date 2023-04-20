/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcResourceLevelRelationship.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcMonetaryUnit;
	class IFCQUERY_EXPORT IfcPositiveRatioMeasure;
	class IFCQUERY_EXPORT IfcDateTime;
	class IFCQUERY_EXPORT IfcLibraryInformation;
	//ENTITY
	class IFCQUERY_EXPORT IfcCurrencyRelationship : public IfcResourceLevelRelationship
	{ 
	public:
		IfcCurrencyRelationship() = default;
		IfcCurrencyRelationship( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 7; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 539742890; }

		// IfcResourceLevelRelationship -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcLabel>				m_Name;						//optional
		//  shared_ptr<IfcText>					m_Description;				//optional

		// IfcCurrencyRelationship -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcMonetaryUnit>			m_RelatingMonetaryUnit;
		shared_ptr<IfcMonetaryUnit>			m_RelatedMonetaryUnit;
		shared_ptr<IfcPositiveRatioMeasure>	m_ExchangeRate;
		shared_ptr<IfcDateTime>				m_RateDateTime;				//optional
		shared_ptr<IfcLibraryInformation>	m_RateSource;				//optional
	};
}
