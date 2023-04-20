/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcArbitraryClosedProfileDef.h"
#include "ifcpp/Ifc/IfcCurve.h"
#include "ifcpp/Ifc/IfcExternalReferenceRelationship.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcProfileProperties.h"
#include "ifcpp/Ifc/IfcProfileTypeEnum.h"

// ENTITY IfcArbitraryClosedProfileDef 
IFC4X3::IfcArbitraryClosedProfileDef::IfcArbitraryClosedProfileDef( int tag ) { m_tag = tag; }
void IFC4X3::IfcArbitraryClosedProfileDef::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCARBITRARYCLOSEDPROFILEDEF" << "(";
	if( m_ProfileType ) { m_ProfileType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ProfileName ) { m_ProfileName->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OuterCurve ) { stream << "#" << m_OuterCurve->m_tag; } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcArbitraryClosedProfileDef::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcArbitraryClosedProfileDef::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcArbitraryClosedProfileDef, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_ProfileType = IfcProfileTypeEnum::createObjectFromSTEP( args[0], map, errorStream );
	m_ProfileName = IfcLabel::createObjectFromSTEP( args[1], map, errorStream );
	readEntityReference( args[2], m_OuterCurve, map, errorStream );
}
void IFC4X3::IfcArbitraryClosedProfileDef::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcProfileDef::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "OuterCurve", m_OuterCurve ) );
}
void IFC4X3::IfcArbitraryClosedProfileDef::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcProfileDef::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcArbitraryClosedProfileDef::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcProfileDef::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcArbitraryClosedProfileDef::unlinkFromInverseCounterparts()
{
	IfcProfileDef::unlinkFromInverseCounterparts();
}