/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcResourceObjectSelect.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcRoleEnum;
	class IFCQUERY_EXPORT IfcLabel;
	class IFCQUERY_EXPORT IfcText;
	class IFCQUERY_EXPORT IfcExternalReferenceRelationship;
	//ENTITY
	class IFCQUERY_EXPORT IfcActorRole : virtual public IfcResourceObjectSelect, public BuildingEntity
	{
	public:
		IfcActorRole() = default;
		IfcActorRole( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 3; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 3630933823; }

		// IfcActorRole -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcRoleEnum>										m_Role;
		shared_ptr<IfcLabel>										m_UserDefinedRole;			//optional
		shared_ptr<IfcText>											m_Description;				//optional
		// inverse attributes:
		std::vector<weak_ptr<IfcExternalReferenceRelationship> >	m_HasExternalReference_inverse;
	};
}
