/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcCurveStyleFontAndScaling.h"
#include "ifcpp/Ifc/IfcCurveStyleFontSelect.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcPositiveRatioMeasure.h"

// ENTITY IfcCurveStyleFontAndScaling 
IFC4X3::IfcCurveStyleFontAndScaling::IfcCurveStyleFontAndScaling( int tag ) { m_tag = tag; }
void IFC4X3::IfcCurveStyleFontAndScaling::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCCURVESTYLEFONTANDSCALING" << "(";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CurveStyleFont ) { m_CurveStyleFont->getStepParameter( stream, true ); } else { stream << "$" ; }
	stream << ",";
	if( m_CurveFontScaling ) { m_CurveFontScaling->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcCurveStyleFontAndScaling::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcCurveStyleFontAndScaling::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcCurveStyleFontAndScaling, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Name = IfcLabel::createObjectFromSTEP( args[0], map, errorStream );
	m_CurveStyleFont = IfcCurveStyleFontSelect::createObjectFromSTEP( args[1], map, errorStream );
	m_CurveFontScaling = IfcPositiveRatioMeasure::createObjectFromSTEP( args[2], map, errorStream );
}
void IFC4X3::IfcCurveStyleFontAndScaling::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcPresentationItem::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "Name", m_Name ) );
	vec_attributes.emplace_back( std::make_pair( "CurveStyleFont", m_CurveStyleFont ) );
	vec_attributes.emplace_back( std::make_pair( "CurveFontScaling", m_CurveFontScaling ) );
}
void IFC4X3::IfcCurveStyleFontAndScaling::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcPresentationItem::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcCurveStyleFontAndScaling::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcPresentationItem::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcCurveStyleFontAndScaling::unlinkFromInverseCounterparts()
{
	IfcPresentationItem::unlinkFromInverseCounterparts();
}
