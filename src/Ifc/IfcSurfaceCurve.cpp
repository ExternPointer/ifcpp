/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcCurve.h"
#include "ifcpp/Ifc/IfcPcurve.h"
#include "ifcpp/Ifc/IfcPreferredSurfaceCurveRepresentation.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSurfaceCurve.h"

// ENTITY IfcSurfaceCurve 
IFC4X3::IfcSurfaceCurve::IfcSurfaceCurve( int tag ) { m_tag = tag; }
void IFC4X3::IfcSurfaceCurve::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSURFACECURVE" << "(";
	if( m_Curve3D ) { stream << "#" << m_Curve3D->m_tag; } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_AssociatedGeometry );
	stream << ",";
	if( m_MasterRepresentation ) { m_MasterRepresentation->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcSurfaceCurve::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSurfaceCurve::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSurfaceCurve, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_Curve3D, map, errorStream );
	readEntityReferenceList( args[1], m_AssociatedGeometry, map, errorStream );
	m_MasterRepresentation = IfcPreferredSurfaceCurveRepresentation::createObjectFromSTEP( args[2], map, errorStream );
}
void IFC4X3::IfcSurfaceCurve::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcCurve::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Curve3D", m_Curve3D ) );
	if( !m_AssociatedGeometry.empty() )
	{
		shared_ptr<AttributeObjectVector> AssociatedGeometry_vec_object( new AttributeObjectVector() );
		std::copy( m_AssociatedGeometry.begin(), m_AssociatedGeometry.end(), std::back_inserter( AssociatedGeometry_vec_object->m_vec ) );
		vec_attributes.emplace_back( std::make_pair( "AssociatedGeometry", AssociatedGeometry_vec_object ) );
	}
	vec_attributes.emplace_back( std::make_pair( "MasterRepresentation", m_MasterRepresentation ) );
}
void IFC4X3::IfcSurfaceCurve::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcCurve::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcSurfaceCurve::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcCurve::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSurfaceCurve::unlinkFromInverseCounterparts()
{
	IfcCurve::unlinkFromInverseCounterparts();
}