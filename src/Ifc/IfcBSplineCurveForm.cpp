/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcBSplineCurveForm.h"

// TYPE IfcBSplineCurveForm = ENUMERATION OF	(CIRCULAR_ARC	,ELLIPTIC_ARC	,HYPERBOLIC_ARC	,PARABOLIC_ARC	,POLYLINE_FORM	,UNSPECIFIED);
void IFC4X3::IfcBSplineCurveForm::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCBSPLINECURVEFORM("; }
	switch( m_enum )
	{
		case ENUM_CIRCULAR_ARC:	stream << ".CIRCULAR_ARC."; break;
		case ENUM_ELLIPTIC_ARC:	stream << ".ELLIPTIC_ARC."; break;
		case ENUM_HYPERBOLIC_ARC:	stream << ".HYPERBOLIC_ARC."; break;
		case ENUM_PARABOLIC_ARC:	stream << ".PARABOLIC_ARC."; break;
		case ENUM_POLYLINE_FORM:	stream << ".POLYLINE_FORM."; break;
		case ENUM_UNSPECIFIED:	stream << ".UNSPECIFIED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcBSplineCurveForm> IFC4X3::IfcBSplineCurveForm::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcBSplineCurveForm>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcBSplineCurveForm>(); }
	shared_ptr<IfcBSplineCurveForm> type_object( new IfcBSplineCurveForm() );
	if( std_iequal( arg, ".CIRCULAR_ARC." ) )
	{
		type_object->m_enum = IfcBSplineCurveForm::ENUM_CIRCULAR_ARC;
	}
	else if( std_iequal( arg, ".ELLIPTIC_ARC." ) )
	{
		type_object->m_enum = IfcBSplineCurveForm::ENUM_ELLIPTIC_ARC;
	}
	else if( std_iequal( arg, ".HYPERBOLIC_ARC." ) )
	{
		type_object->m_enum = IfcBSplineCurveForm::ENUM_HYPERBOLIC_ARC;
	}
	else if( std_iequal( arg, ".PARABOLIC_ARC." ) )
	{
		type_object->m_enum = IfcBSplineCurveForm::ENUM_PARABOLIC_ARC;
	}
	else if( std_iequal( arg, ".POLYLINE_FORM." ) )
	{
		type_object->m_enum = IfcBSplineCurveForm::ENUM_POLYLINE_FORM;
	}
	else if( std_iequal( arg, ".UNSPECIFIED." ) )
	{
		type_object->m_enum = IfcBSplineCurveForm::ENUM_UNSPECIFIED;
	}
	return type_object;
}
