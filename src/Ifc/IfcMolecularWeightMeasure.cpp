/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcDerivedMeasureValue.h"
#include "ifcpp/Ifc/IfcMolecularWeightMeasure.h"

// TYPE IfcMolecularWeightMeasure = REAL;
IFC4X3::IfcMolecularWeightMeasure::IfcMolecularWeightMeasure( double value ) { m_value = value; }
void IFC4X3::IfcMolecularWeightMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCMOLECULARWEIGHTMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcMolecularWeightMeasure> IFC4X3::IfcMolecularWeightMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcMolecularWeightMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcMolecularWeightMeasure>(); }
	shared_ptr<IfcMolecularWeightMeasure> type_object( new IfcMolecularWeightMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}