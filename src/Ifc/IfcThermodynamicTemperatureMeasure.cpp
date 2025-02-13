/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcThermodynamicTemperatureMeasure.h"

// TYPE IfcThermodynamicTemperatureMeasure = REAL;
IFC4X3::IfcThermodynamicTemperatureMeasure::IfcThermodynamicTemperatureMeasure( double value ) { m_value = value; }
void IFC4X3::IfcThermodynamicTemperatureMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCTHERMODYNAMICTEMPERATUREMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcThermodynamicTemperatureMeasure> IFC4X3::IfcThermodynamicTemperatureMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcThermodynamicTemperatureMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcThermodynamicTemperatureMeasure>(); }
	shared_ptr<IfcThermodynamicTemperatureMeasure> type_object( new IfcThermodynamicTemperatureMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}
