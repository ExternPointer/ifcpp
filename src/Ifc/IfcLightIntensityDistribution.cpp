/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcLightDistributionCurveEnum.h"
#include "ifcpp/Ifc/IfcLightDistributionData.h"
#include "ifcpp/Ifc/IfcLightIntensityDistribution.h"

// ENTITY IfcLightIntensityDistribution 
IFC4X3::IfcLightIntensityDistribution::IfcLightIntensityDistribution( int tag ) { m_tag = tag; }
void IFC4X3::IfcLightIntensityDistribution::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCLIGHTINTENSITYDISTRIBUTION" << "(";
	if( m_LightDistributionCurve ) { m_LightDistributionCurve->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_DistributionData );
	stream << ");";
}
void IFC4X3::IfcLightIntensityDistribution::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcLightIntensityDistribution::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 2 ){ std::stringstream err; err << "Wrong parameter count for entity IfcLightIntensityDistribution, expecting 2, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_LightDistributionCurve = IfcLightDistributionCurveEnum::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReferenceList( args[1], m_DistributionData, map, errorStream );
}
void IFC4X3::IfcLightIntensityDistribution::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "LightDistributionCurve", m_LightDistributionCurve ) );
	shared_ptr<AttributeObjectVector> DistributionData_vec_object( new AttributeObjectVector() );
	std::copy( m_DistributionData.begin(), m_DistributionData.end(), std::back_inserter( DistributionData_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "DistributionData", DistributionData_vec_object ) );
}
void IFC4X3::IfcLightIntensityDistribution::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
}
void IFC4X3::IfcLightIntensityDistribution::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcLightIntensityDistribution::unlinkFromInverseCounterparts()
{
}
