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
	class IFCQUERY_EXPORT IfcResourceObjectSelect;
	class IFCQUERY_EXPORT IfcApproval;
	//ENTITY
	class IFCQUERY_EXPORT IfcResourceApprovalRelationship : public IfcResourceLevelRelationship
	{
	public:
		IfcResourceApprovalRelationship() = default;
		IfcResourceApprovalRelationship( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 4; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 2943643501; }

		// IfcResourceLevelRelationship -----------------------------------------------------------
		// attributes:
		//  shared_ptr<IfcLabel>								m_Name;						//optional
		//  shared_ptr<IfcText>									m_Description;				//optional

		// IfcResourceApprovalRelationship -----------------------------------------------------------
		// attributes:
		std::vector<shared_ptr<IfcResourceObjectSelect> >	m_RelatedResourceObjects;
		shared_ptr<IfcApproval>								m_RelatingApproval;
	};
}
