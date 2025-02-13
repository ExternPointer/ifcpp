/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcIdentifier;
	class IFCQUERY_EXPORT IfcLabel;
	class IFCQUERY_EXPORT IfcText;
	class IFCQUERY_EXPORT IfcUnit;
	class IFCQUERY_EXPORT IfcReference;
	//ENTITY
	class IFCQUERY_EXPORT IfcTableColumn : public BuildingEntity
	{
	public:
		IfcTableColumn() = default;
		IfcTableColumn( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 5; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 2043862942; }

		// IfcTableColumn -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcIdentifier>	m_Identifier;				//optional
		shared_ptr<IfcLabel>		m_Name;						//optional
		shared_ptr<IfcText>			m_Description;				//optional
		shared_ptr<IfcUnit>			m_Unit;						//optional
		shared_ptr<IfcReference>	m_ReferencePath;			//optional
	};
}
