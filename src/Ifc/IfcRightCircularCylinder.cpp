/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcRightCircularCylinder.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcRightCircularCylinder 
IFC4X3::IfcRightCircularCylinder::IfcRightCircularCylinder( int tag ) { m_tag = tag; }
void IFC4X3::IfcRightCircularCylinder::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCRIGHTCIRCULARCYLINDER" << "(";
	if( m_Position ) { stream << "#" << m_Position->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Height ) { m_Height->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Radius ) { m_Radius->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcRightCircularCylinder::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcRightCircularCylinder::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcRightCircularCylinder, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_Position, map, errorStream );
	m_Height = IfcPositiveLengthMeasure::createObjectFromSTEP( args[1], map, errorStream );
	m_Radius = IfcPositiveLengthMeasure::createObjectFromSTEP( args[2], map, errorStream );
}
void IFC4X3::IfcRightCircularCylinder::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcCsgPrimitive3D::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Height", m_Height ) );
	vec_attributes.emplace_back( std::make_pair( "Radius", m_Radius ) );
}
void IFC4X3::IfcRightCircularCylinder::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcCsgPrimitive3D::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcRightCircularCylinder::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcCsgPrimitive3D::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcRightCircularCylinder::unlinkFromInverseCounterparts()
{
	IfcCsgPrimitive3D::unlinkFromInverseCounterparts();
}
