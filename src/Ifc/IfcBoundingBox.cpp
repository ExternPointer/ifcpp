/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcBoundingBox.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcBoundingBox 
IFC4X3::IfcBoundingBox::IfcBoundingBox( int tag ) { m_tag = tag; }
void IFC4X3::IfcBoundingBox::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCBOUNDINGBOX" << "(";
	if( m_Corner ) { stream << "#" << m_Corner->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_XDim ) { m_XDim->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_YDim ) { m_YDim->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ZDim ) { m_ZDim->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcBoundingBox::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcBoundingBox::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcBoundingBox, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_Corner, map, errorStream );
	m_XDim = IfcPositiveLengthMeasure::createObjectFromSTEP( args[1], map, errorStream );
	m_YDim = IfcPositiveLengthMeasure::createObjectFromSTEP( args[2], map, errorStream );
	m_ZDim = IfcPositiveLengthMeasure::createObjectFromSTEP( args[3], map, errorStream );
}
void IFC4X3::IfcBoundingBox::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Corner", m_Corner ) );
	vec_attributes.emplace_back( std::make_pair( "XDim", m_XDim ) );
	vec_attributes.emplace_back( std::make_pair( "YDim", m_YDim ) );
	vec_attributes.emplace_back( std::make_pair( "ZDim", m_ZDim ) );
}
void IFC4X3::IfcBoundingBox::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcGeometricRepresentationItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcBoundingBox::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcGeometricRepresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcBoundingBox::unlinkFromInverseCounterparts()
{
	IfcGeometricRepresentationItem::unlinkFromInverseCounterparts();
}
