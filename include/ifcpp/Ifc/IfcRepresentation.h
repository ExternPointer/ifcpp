/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcLayeredItem.h"
namespace IFC4X3
{
	class IFCQUERY_EXPORT IfcRepresentationContext;
	class IFCQUERY_EXPORT IfcLabel;
	class IFCQUERY_EXPORT IfcRepresentationItem;
	class IFCQUERY_EXPORT IfcRepresentationMap;
	class IFCQUERY_EXPORT IfcPresentationLayerAssignment;
	class IFCQUERY_EXPORT IfcProductRepresentation;
	//ENTITY
	class IFCQUERY_EXPORT IfcRepresentation : virtual public IfcLayeredItem, public BuildingEntity
	{
	public:
		IfcRepresentation() = default;
		IfcRepresentation( int id );
		virtual void getStepLine( std::stringstream& stream ) const;
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		virtual void readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		virtual void setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self );
		virtual uint8_t getNumAttributes() const { return 4; }
		virtual void getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const;
		virtual void unlinkFromInverseCounterparts();
		virtual uint32_t classID() const { return 1076942058; }

		// IfcRepresentation -----------------------------------------------------------
		// attributes:
		shared_ptr<IfcRepresentationContext>					m_ContextOfItems;
		shared_ptr<IfcLabel>									m_RepresentationIdentifier;	//optional
		shared_ptr<IfcLabel>									m_RepresentationType;		//optional
		std::vector<shared_ptr<IfcRepresentationItem> >			m_Items;
		// inverse attributes:
		std::vector<weak_ptr<IfcRepresentationMap> >			m_RepresentationMap_inverse;
		std::vector<weak_ptr<IfcPresentationLayerAssignment> >	m_LayerAssignments_inverse;
		std::vector<weak_ptr<IfcProductRepresentation> >		m_OfProductRepresentation_inverse;
	};
}
