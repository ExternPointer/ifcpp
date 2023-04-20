/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcDerivedMeasureValue.h"
#include "ifcpp/Ifc/IfcRotationalMassMeasure.h"

// TYPE IfcRotationalMassMeasure = REAL;
IFC4X3::IfcRotationalMassMeasure::IfcRotationalMassMeasure( double value ) { m_value = value; }
void IFC4X3::IfcRotationalMassMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCROTATIONALMASSMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcRotationalMassMeasure> IFC4X3::IfcRotationalMassMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcRotationalMassMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcRotationalMassMeasure>(); }
	shared_ptr<IfcRotationalMassMeasure> type_object( new IfcRotationalMassMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}