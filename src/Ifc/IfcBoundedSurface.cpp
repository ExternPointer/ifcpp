/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoundedSurface.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcBoundedSurface 
IFC4X3::IfcBoundedSurface::IfcBoundedSurface( int tag ) { m_tag = tag; }
void IFC4X3::IfcBoundedSurface::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCBOUNDEDSURFACE" << "(";
	stream << ");";
}
void IFC4X3::IfcBoundedSurface::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcBoundedSurface::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
}
void IFC4X3::IfcBoundedSurface::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSurface::getAttributes( vec_attributes );
}
void IFC4X3::IfcBoundedSurface::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSurface::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcBoundedSurface::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSurface::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcBoundedSurface::unlinkFromInverseCounterparts()
{
	IfcSurface::unlinkFromInverseCounterparts();
}
