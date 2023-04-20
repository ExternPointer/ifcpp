/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcConnectionGeometry.h"

// ENTITY IfcConnectionGeometry 
IFC4X3::IfcConnectionGeometry::IfcConnectionGeometry( int tag ) { m_tag = tag; }
void IFC4X3::IfcConnectionGeometry::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCONNECTIONGEOMETRY" << "(";
	stream << ");";
}
void IFC4X3::IfcConnectionGeometry::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcConnectionGeometry::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
}
void IFC4X3::IfcConnectionGeometry::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
}
void IFC4X3::IfcConnectionGeometry::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
}
void IFC4X3::IfcConnectionGeometry::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcConnectionGeometry::unlinkFromInverseCounterparts()
{
}