/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/reader/ReaderUtil.h"
#include "ifcpp/writer/WriterUtil.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingException.h"
#include "ifcpp/IFC4X3/include/IfcSurfaceFeatureTypeEnum.h"

// TYPE IfcSurfaceFeatureTypeEnum = ENUMERATION OF	(DEFECT	,HATCHMARKING	,LINEMARKING	,MARK	,NONSKIDSURFACING	,PAVEMENTSURFACEMARKING	,RUMBLESTRIP	,SYMBOLMARKING	,TAG	,TRANSVERSERUMBLESTRIP	,TREATMENT	,USERDEFINED	,NOTDEFINED);
void IFC4X3::IfcSurfaceFeatureTypeEnum::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCSURFACEFEATURETYPEENUM("; }
	switch( m_enum )
	{
		case ENUM_DEFECT:	stream << ".DEFECT."; break;
		case ENUM_HATCHMARKING:	stream << ".HATCHMARKING."; break;
		case ENUM_LINEMARKING:	stream << ".LINEMARKING."; break;
		case ENUM_MARK:	stream << ".MARK."; break;
		case ENUM_NONSKIDSURFACING:	stream << ".NONSKIDSURFACING."; break;
		case ENUM_PAVEMENTSURFACEMARKING:	stream << ".PAVEMENTSURFACEMARKING."; break;
		case ENUM_RUMBLESTRIP:	stream << ".RUMBLESTRIP."; break;
		case ENUM_SYMBOLMARKING:	stream << ".SYMBOLMARKING."; break;
		case ENUM_TAG:	stream << ".TAG."; break;
		case ENUM_TRANSVERSERUMBLESTRIP:	stream << ".TRANSVERSERUMBLESTRIP."; break;
		case ENUM_TREATMENT:	stream << ".TREATMENT."; break;
		case ENUM_USERDEFINED:	stream << ".USERDEFINED."; break;
		case ENUM_NOTDEFINED:	stream << ".NOTDEFINED."; break;
	}
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcSurfaceFeatureTypeEnum> IFC4X3::IfcSurfaceFeatureTypeEnum::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcSurfaceFeatureTypeEnum>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcSurfaceFeatureTypeEnum>(); }
	shared_ptr<IfcSurfaceFeatureTypeEnum> type_object( new IfcSurfaceFeatureTypeEnum() );
	if( std_iequal( arg, ".DEFECT." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_DEFECT;
	}
	else if( std_iequal( arg, ".HATCHMARKING." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_HATCHMARKING;
	}
	else if( std_iequal( arg, ".LINEMARKING." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_LINEMARKING;
	}
	else if( std_iequal( arg, ".MARK." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_MARK;
	}
	else if( std_iequal( arg, ".NONSKIDSURFACING." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_NONSKIDSURFACING;
	}
	else if( std_iequal( arg, ".PAVEMENTSURFACEMARKING." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_PAVEMENTSURFACEMARKING;
	}
	else if( std_iequal( arg, ".RUMBLESTRIP." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_RUMBLESTRIP;
	}
	else if( std_iequal( arg, ".SYMBOLMARKING." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_SYMBOLMARKING;
	}
	else if( std_iequal( arg, ".TAG." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_TAG;
	}
	else if( std_iequal( arg, ".TRANSVERSERUMBLESTRIP." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_TRANSVERSERUMBLESTRIP;
	}
	else if( std_iequal( arg, ".TREATMENT." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_TREATMENT;
	}
	else if( std_iequal( arg, ".USERDEFINED." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_USERDEFINED;
	}
	else if( std_iequal( arg, ".NOTDEFINED." ) )
	{
		type_object->m_enum = IfcSurfaceFeatureTypeEnum::ENUM_NOTDEFINED;
	}
	return type_object;
}
