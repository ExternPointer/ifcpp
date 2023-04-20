/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcDerivedMeasureValue.h"
#include "ifcpp/Ifc/IfcIonConcentrationMeasure.h"

// TYPE IfcIonConcentrationMeasure = REAL;
IFC4X3::IfcIonConcentrationMeasure::IfcIonConcentrationMeasure( double value ) { m_value = value; }
void IFC4X3::IfcIonConcentrationMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCIONCONCENTRATIONMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcIonConcentrationMeasure> IFC4X3::IfcIonConcentrationMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcIonConcentrationMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcIonConcentrationMeasure>(); }
	shared_ptr<IfcIonConcentrationMeasure> type_object( new IfcIonConcentrationMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}