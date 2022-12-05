/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/model/AttributeObject.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/model/BuildingGuid.h"
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/IFC4X3/include/IfcForceMeasure.h"
#include "ifcpp/IFC4X3/include/IfcLabel.h"
#include "ifcpp/IFC4X3/include/IfcStructuralLoadSingleForceWarping.h"
#include "ifcpp/IFC4X3/include/IfcTorqueMeasure.h"
#include "ifcpp/IFC4X3/include/IfcWarpingMomentMeasure.h"

// ENTITY IfcStructuralLoadSingleForceWarping 
IFC4X3::IfcStructuralLoadSingleForceWarping::IfcStructuralLoadSingleForceWarping( int tag ) { m_tag = tag; }
void IFC4X3::IfcStructuralLoadSingleForceWarping::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSTRUCTURALLOADSINGLEFORCEWARPING" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ForceX ) { m_ForceX->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ForceY ) { m_ForceY->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ForceZ ) { m_ForceZ->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_MomentX ) { m_MomentX->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_MomentY ) { m_MomentY->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_MomentZ ) { m_MomentZ->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_WarpingMoment ) { m_WarpingMoment->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcStructuralLoadSingleForceWarping::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcStructuralLoadSingleForceWarping::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 8 ){ std::stringstream err; err << "Wrong parameter count for entity IfcStructuralLoadSingleForceWarping, expecting 8, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_ForceX = IfcForceMeasure::createObjectFromSTEP( args[1], map, errorStream );
	m_ForceY = IfcForceMeasure::createObjectFromSTEP( args[2], map, errorStream );
	m_ForceZ = IfcForceMeasure::createObjectFromSTEP( args[3], map, errorStream );
	m_MomentX = IfcTorqueMeasure::createObjectFromSTEP( args[4], map, errorStream );
	m_MomentY = IfcTorqueMeasure::createObjectFromSTEP( args[5], map, errorStream );
	m_MomentZ = IfcTorqueMeasure::createObjectFromSTEP( args[6], map, errorStream );
	m_WarpingMoment = IfcWarpingMomentMeasure::createObjectFromSTEP( args[7], map, errorStream );
}
void IFC4X3::IfcStructuralLoadSingleForceWarping::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcStructuralLoadSingleForce::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "WarpingMoment", m_WarpingMoment ) );
}
void IFC4X3::IfcStructuralLoadSingleForceWarping::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcStructuralLoadSingleForce::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcStructuralLoadSingleForceWarping::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcStructuralLoadSingleForce::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcStructuralLoadSingleForceWarping::unlinkFromInverseCounterparts()
{
	IfcStructuralLoadSingleForce::unlinkFromInverseCounterparts();
}
