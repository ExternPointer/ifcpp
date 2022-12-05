/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcBSplineSurfaceForm.h"

// TYPE IfcBSplineSurfaceForm = ENUMERATION OF	(CONICAL_SURF	,CYLINDRICAL_SURF	,GENERALISED_CONE	,PLANE_SURF	,QUADRIC_SURF	,RULED_SURF	,SPHERICAL_SURF	,SURF_OF_LINEAR_EXTRUSION	,SURF_OF_REVOLUTION	,TOROIDAL_SURF	,UNSPECIFIED);
void IFC4X3::IfcBSplineSurfaceForm::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCBSPLINESURFACEFORM("; }
	switch( m_enum )
	{
		case ENUM_CONICAL_SURF:	stream << ".CONICAL_SURF."; break;
		case ENUM_CYLINDRICAL_SURF:	stream << ".CYLINDRICAL_SURF."; break;
		case ENUM_GENERALISED_CONE:	stream << ".GENERALISED_CONE."; break;
		case ENUM_PLANE_SURF:	stream << ".PLANE_SURF."; break;
		case ENUM_QUADRIC_SURF:	stream << ".QUADRIC_SURF."; break;
		case ENUM_RULED_SURF:	stream << ".RULED_SURF."; break;
		case ENUM_SPHERICAL_SURF:	stream << ".SPHERICAL_SURF."; break;
		case ENUM_SURF_OF_LINEAR_EXTRUSION:	stream << ".SURF_OF_LINEAR_EXTRUSION."; break;
		case ENUM_SURF_OF_REVOLUTION:	stream << ".SURF_OF_REVOLUTION."; break;
		case ENUM_TOROIDAL_SURF:	stream << ".TOROIDAL_SURF."; break;
		case ENUM_UNSPECIFIED:	stream << ".UNSPECIFIED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcBSplineSurfaceForm> IFC4X3::IfcBSplineSurfaceForm::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcBSplineSurfaceForm>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcBSplineSurfaceForm>(); }
	shared_ptr<IfcBSplineSurfaceForm> type_object( new IfcBSplineSurfaceForm() );
	if( std_iequal( arg, ".CONICAL_SURF." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_CONICAL_SURF;
	}
	else if( std_iequal( arg, ".CYLINDRICAL_SURF." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_CYLINDRICAL_SURF;
	}
	else if( std_iequal( arg, ".GENERALISED_CONE." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_GENERALISED_CONE;
	}
	else if( std_iequal( arg, ".PLANE_SURF." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_PLANE_SURF;
	}
	else if( std_iequal( arg, ".QUADRIC_SURF." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_QUADRIC_SURF;
	}
	else if( std_iequal( arg, ".RULED_SURF." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_RULED_SURF;
	}
	else if( std_iequal( arg, ".SPHERICAL_SURF." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_SPHERICAL_SURF;
	}
	else if( std_iequal( arg, ".SURF_OF_LINEAR_EXTRUSION." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_SURF_OF_LINEAR_EXTRUSION;
	}
	else if( std_iequal( arg, ".SURF_OF_REVOLUTION." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_SURF_OF_REVOLUTION;
	}
	else if( std_iequal( arg, ".TOROIDAL_SURF." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_TOROIDAL_SURF;
	}
	else if( std_iequal( arg, ".UNSPECIFIED." ) )
	{
		type_object->m_enum = IfcBSplineSurfaceForm::ENUM_UNSPECIFIED;
	}
	return type_object;
}
