/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcCartesianPoint.h"
#include "ifcpp/Ifc/IfcCartesianTransformationOperator3DnonUniform.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcReal.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcCartesianTransformationOperator3DnonUniform 
IFC4X3::IfcCartesianTransformationOperator3DnonUniform::IfcCartesianTransformationOperator3DnonUniform( int tag ) { m_tag = tag; }
void IFC4X3::IfcCartesianTransformationOperator3DnonUniform::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM" << "(";
	if( m_Axis1 ) { stream << "#" << m_Axis1->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Axis2 ) { stream << "#" << m_Axis2->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_LocalOrigin ) { stream << "#" << m_LocalOrigin->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Scale ) { m_Scale->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Axis3 ) { stream << "#" << m_Axis3->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Scale2 ) { m_Scale2->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Scale3 ) { m_Scale3->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcCartesianTransformationOperator3DnonUniform::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcCartesianTransformationOperator3DnonUniform::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 7 ){ std::stringstream err; err << "Wrong parameter count for entity IfcCartesianTransformationOperator3DnonUniform, expecting 7, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_Axis1, map, errorStream );
	readEntityReference( args[1], m_Axis2, map, errorStream );
	readEntityReference( args[2], m_LocalOrigin, map, errorStream );
	m_Scale = IfcReal::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReference( args[4], m_Axis3, map, errorStream );
	m_Scale2 = IfcReal::createObjectFromSTEP( args[5], map, errorStream );
	m_Scale3 = IfcReal::createObjectFromSTEP( args[6], map, errorStream );
}
void IFC4X3::IfcCartesianTransformationOperator3DnonUniform::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcCartesianTransformationOperator3D::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Scale2", m_Scale2 ) );
	vec_attributes.emplace_back( std::make_pair( "Scale3", m_Scale3 ) );
}
void IFC4X3::IfcCartesianTransformationOperator3DnonUniform::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcCartesianTransformationOperator3D::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcCartesianTransformationOperator3DnonUniform::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcCartesianTransformationOperator3D::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcCartesianTransformationOperator3DnonUniform::unlinkFromInverseCounterparts()
{
	IfcCartesianTransformationOperator3D::unlinkFromInverseCounterparts();
}
