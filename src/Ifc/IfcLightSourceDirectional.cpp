/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcColourRgb.h"
#include "ifcpp/Ifc/IfcDirection.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcLightSourceDirectional.h"
#include "ifcpp/Ifc/IfcNormalisedRatioMeasure.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcStyledItem.h"

// ENTITY IfcLightSourceDirectional 
IFC4X3::IfcLightSourceDirectional::IfcLightSourceDirectional( int tag ) { m_tag = tag; }
void IFC4X3::IfcLightSourceDirectional::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCLIGHTSOURCEDIRECTIONAL" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_LightColour ) { stream << "#" << m_LightColour->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_AmbientIntensity ) { m_AmbientIntensity->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Intensity ) { m_Intensity->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Orientation ) { stream << "#" << m_Orientation->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcLightSourceDirectional::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcLightSourceDirectional::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 5 ){ std::stringstream err; err << "Wrong parameter count for entity IfcLightSourceDirectional, expecting 5, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_LightColour, map, errorStream );
	m_AmbientIntensity = IfcNormalisedRatioMeasure::createObjectFromSTEP( args[2], map, errorStream );
	m_Intensity = IfcNormalisedRatioMeasure::createObjectFromSTEP( args[3], map, errorStream );
	readEntityReference( args[4], m_Orientation, map, errorStream );
}
void IFC4X3::IfcLightSourceDirectional::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcLightSource::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Orientation", m_Orientation ) );
}
void IFC4X3::IfcLightSourceDirectional::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcLightSource::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcLightSourceDirectional::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcLightSource::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcLightSourceDirectional::unlinkFromInverseCounterparts()
{
	IfcLightSource::unlinkFromInverseCounterparts();
}
