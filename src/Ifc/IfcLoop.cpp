/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcLoop.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcLoop 
IFC4X3::IfcLoop::IfcLoop( int tag ) { m_tag = tag; }
void IFC4X3::IfcLoop::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCLOOP" << "(";
	stream << ");";
}
void IFC4X3::IfcLoop::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcLoop::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
}
void IFC4X3::IfcLoop::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcTopologicalRepresentationItem::getAttributes( vec_attributes );
}
void IFC4X3::IfcLoop::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcTopologicalRepresentationItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcLoop::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcTopologicalRepresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcLoop::unlinkFromInverseCounterparts()
{
	IfcTopologicalRepresentationItem::unlinkFromInverseCounterparts();
}