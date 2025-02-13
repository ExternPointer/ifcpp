/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcCurveStyleFont.h"
#include "ifcpp/Ifc/IfcCurveStyleFontPattern.h"
#include "ifcpp/Ifc/IfcLabel.h"

// ENTITY IfcCurveStyleFont 
IFC4X3::IfcCurveStyleFont::IfcCurveStyleFont( int tag ) { m_tag = tag; }
void IFC4X3::IfcCurveStyleFont::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCURVESTYLEFONT" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_PatternList );
	stream << ");";
}
void IFC4X3::IfcCurveStyleFont::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcCurveStyleFont::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 2 ){ std::stringstream err; err << "Wrong parameter count for entity IfcCurveStyleFont, expecting 2, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReferenceList( args[1], m_PatternList, map, errorStream );
}
void IFC4X3::IfcCurveStyleFont::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPresentationItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
	shared_ptr<AttributeObjectVector> PatternList_vec_object( new AttributeObjectVector() );
	std::copy( m_PatternList.begin(), m_PatternList.end(), std::back_inserter( PatternList_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "PatternList", PatternList_vec_object ) );
}
void IFC4X3::IfcCurveStyleFont::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPresentationItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcCurveStyleFont::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcCurveStyleFont::unlinkFromInverseCounterparts()
{
	IfcPresentationItem::unlinkFromInverseCounterparts();
}
