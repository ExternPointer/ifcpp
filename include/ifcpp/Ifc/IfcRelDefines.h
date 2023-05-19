/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcRelationship.h"
namespace IFC4X3
{
	//ENTITY
	class IFCQUERY_EXPORT IfcRelDefines : public IfcRelationship
	{
	public:
		IfcRelDefines() = default;
		IfcRelDefines( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 4; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 693640335; }

		// IfcRoot -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcGloballyUniqueId>	m_GlobalId;
		//  shared_ptr<IfcOwnerHistory>		m_OwnerHistory;				//optional
		//  shared_ptr<IfcLabel>			m_Name;						//optional
		//  shared_ptr<IfcText>				m_Description;				//optional

		// IfcRelationship -----------------------------------------------------------

		// IfcRelDefines -----------------------------------------------------------
	};
}
