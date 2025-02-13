/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAxis2Placement3D.h"
#include "ifcpp/Ifc/IfcPresentationLayerAssignment.h"
#include "ifcpp/Ifc/IfcProfileDef.h"
#include "ifcpp/Ifc/IfcStyledItem.h"
#include "ifcpp/Ifc/IfcSweptAreaSolid.h"

// ENTITY IfcSweptAreaSolid 
IFC4X3::IfcSweptAreaSolid::IfcSweptAreaSolid( int tag ) { m_tag = tag; }
void IFC4X3::IfcSweptAreaSolid::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCSWEPTAREASOLID" << "(";
	if( m_SweptArea ) { stream << "#" << m_SweptArea->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Position ) { stream << "#" << m_Position->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcSweptAreaSolid::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcSweptAreaSolid::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 2 ){ std::stringstream err; err << "Wrong parameter count for entity IfcSweptAreaSolid, expecting 2, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_SweptArea, map, errorStream );
	readEntityReference( args[1], m_Position, map, errorStream );
}
void IFC4X3::IfcSweptAreaSolid::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcSolidModel::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "SweptArea", m_SweptArea ) );
	vec_attributes.emplace_back( std::make_pair( "Position", m_Position ) );
}
void IFC4X3::IfcSweptAreaSolid::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcSolidModel::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcSweptAreaSolid::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcSolidModel::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcSweptAreaSolid::unlinkFromInverseCounterparts()
{
	IfcSolidModel::unlinkFromInverseCounterparts();
}
