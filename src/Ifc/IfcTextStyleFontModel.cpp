/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcFontStyle.h"
#include "ifcpp/Ifc/IfcFontVariant.h"
#include "ifcpp/Ifc/IfcFontWeight.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcSizeSelect.h"
#include "ifcpp/Ifc/IfcTextFontName.h"
#include "ifcpp/Ifc/IfcTextStyleFontModel.h"

// ENTITY IfcTextStyleFontModel 
IFC4X3::IfcTextStyleFontModel::IfcTextStyleFontModel( int tag ) { m_tag = tag; }
void IFC4X3::IfcTextStyleFontModel::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCTEXTSTYLEFONTMODEL" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	stream << "(";
	for( size_t ii = 0; ii < m_FontFamily.size(); ++ii )
	{
		if( ii > 0 )
		{
			stream << ",";
		}
		const shared_ptr<IfcTextFontName>& type_object = m_FontFamily[ii];
		if( type_object )
		{
			type_object->getStepParameter( stream, false );
		}
		else
		{
			stream << "$";
		}
	}
	stream << ")";
	stream << ",";
	if( m_FontStyle ) { m_FontStyle->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_FontVariant ) { m_FontVariant->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_FontWeight ) { m_FontWeight->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_FontSize ) { m_FontSize->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ");";
}
void IFC4X3::IfcTextStyleFontModel::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcTextStyleFontModel::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 6 ){ std::stringstream err; err << "Wrong parameter count for entity IfcTextStyleFontModel, expecting 6, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	readTypeOfStringList( args[1], m_FontFamily );
	m_FontStyle = IfcFontStyle::createObjectFromSTEP( args[2], map, errorStream );
	m_FontVariant = IfcFontVariant::createObjectFromSTEP( args[3], map, errorStream );
	m_FontWeight = IfcFontWeight::createObjectFromSTEP( args[4], map, errorStream );
	m_FontSize = IfcSizeSelect::createObjectFromSTEP( args[5], map, errorStream );
}
void IFC4X3::IfcTextStyleFontModel::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPreDefinedTextFont::getAttributes( vec_attributes );
	shared_ptr<AttributeObjectVector> FontFamily_vec_object( new AttributeObjectVector() );
	std::copy( m_FontFamily.begin(), m_FontFamily.end(), std::back_inserter( FontFamily_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "FontFamily", FontFamily_vec_object ) );
	vec_attributes.emplace_back( std::make_pair( "FontStyle", m_FontStyle ) );
	vec_attributes.emplace_back( std::make_pair( "FontVariant", m_FontVariant ) );
	vec_attributes.emplace_back( std::make_pair( "FontWeight", m_FontWeight ) );
	vec_attributes.emplace_back( std::make_pair( "FontSize", m_FontSize ) );
}
void IFC4X3::IfcTextStyleFontModel::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPreDefinedTextFont::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcTextStyleFontModel::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPreDefinedTextFont::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcTextStyleFontModel::unlinkFromInverseCounterparts()
{
	IfcPreDefinedTextFont::unlinkFromInverseCounterparts();
}
