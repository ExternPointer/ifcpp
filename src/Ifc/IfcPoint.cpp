/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcPoint.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcPoint 
IFC4X3::IfcPoint::IfcPoint( int tag ) { m_tag = tag; }
void IFC4X3::IfcPoint::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPOINT" << "(";
	stream << ");";
}
void IFC4X3::IfcPoint::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcPoint::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
}
void IFC4X3::IfcPoint::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributes( vec_attributes );
}
void IFC4X3::IfcPoint::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcPoint::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcGeometricRepresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcPoint::unlinkFromInverseCounterparts()
{
	IfcGeometricRepresentationItem::unlinkFromInverseCounterparts();
}