/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcCoordinateOperation.h"
#include "ifcpp/Ifc/IfcCoordinateReferenceSystem.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcCoordinateReferenceSystem 
IFC4X3::IfcCoordinateReferenceSystem::IfcCoordinateReferenceSystem( int tag ) { m_tag = tag; }
void IFC4X3::IfcCoordinateReferenceSystem::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCOORDINATEREFERENCESYSTEM" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_GeodeticDatum ) { m_GeodeticDatum->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_VerticalDatum ) { m_VerticalDatum->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcCoordinateReferenceSystem::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcCoordinateReferenceSystem::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 4 ){ std::stringstream err; err << "Wrong parameter count for entity IfcCoordinateReferenceSystem, expecting 4, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	m_GeodeticDatum = IfcIdentifier::createObjectFromSTEP( args[2], map, errorStream );
	m_VerticalDatum = IfcIdentifier::createObjectFromSTEP( args[3], map, errorStream );
}
void IFC4X3::IfcCoordinateReferenceSystem::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
	vec_attributes.emplace_back( std::make_pair( "Description", m_Description ) );
	vec_attributes.emplace_back( std::make_pair( "GeodeticDatum", m_GeodeticDatum ) );
	vec_attributes.emplace_back( std::make_pair( "VerticalDatum", m_VerticalDatum ) );
}
void IFC4X3::IfcCoordinateReferenceSystem::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	shared_ptr<AttributeObjectVector> HasCoordinateOperation_inverse_vec_obj( new AttributeObjectVector() );
	for( size_t i=0; i<m_HasCoordinateOperation_inverse.size(); ++i )
	{
		if( !m_HasCoordinateOperation_inverse[i].expired() )
		{
			HasCoordinateOperation_inverse_vec_obj->m_vec.emplace_back( shared_ptr<IfcCoordinateOperation>( m_HasCoordinateOperation_inverse[i] ) );
		}
	}
	vec_attributes_inverse.emplace_back( std::make_pair( "HasCoordinateOperation_inverse", HasCoordinateOperation_inverse_vec_obj ) );
}
void IFC4X3::IfcCoordinateReferenceSystem::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcCoordinateReferenceSystem::unlinkFromInverseCounterparts()
{
}
