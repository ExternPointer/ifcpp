/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcProductRepresentation.h"
#include "ifcpp/Ifc/IfcRepresentationContext.h"
#include "ifcpp/Ifc/IfcRepresentationItem.h"
#include "ifcpp/Ifc/IfcRepresentationMap.h"
#include "ifcpp/Ifc/IfcShapeAspect.h"
#include "ifcpp/Ifc/IfcTopologyRepresentation.h"

// ENTITY IfcTopologyRepresentation 
IFC4X3::IfcTopologyRepresentation::IfcTopologyRepresentation( int tag ) { m_tag = tag; }
void IFC4X3::IfcTopologyRepresentation::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCTOPOLOGYREPRESENTATION" << "(";
	if( m_ContextOfItems ) { stream << "#" << m_ContextOfItems->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_RepresentationIdentifier ) { m_RepresentationIdentifier->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_RepresentationType ) { m_RepresentationType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_Items );
	stream << ");";
}
void IFC4X3::IfcTopologyRepresentation::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcTopologyRepresentation::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcTopologyRepresentation, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_ContextOfItems, map, errorStream );
	m_RepresentationIdentifier = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	m_RepresentationType = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	readEntityReferenceList( args[3], m_Items, map, errorStream );
}
void IFC4X3::IfcTopologyRepresentation::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcShapeModel::getAttributes( vec_attributes );
}
void IFC4X3::IfcTopologyRepresentation::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcShapeModel::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcTopologyRepresentation::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcShapeModel::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcTopologyRepresentation::unlinkFromInverseCounterparts()
{
	IfcShapeModel::unlinkFromInverseCounterparts();
}