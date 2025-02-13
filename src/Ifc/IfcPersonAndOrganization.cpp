/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcActorRole.h"
#include "ifcpp/Ifc/IfcOrganization.h"
#include "ifcpp/Ifc/IfcPerson.h"
#include "ifcpp/Ifc/IfcPersonAndOrganization.h"

// ENTITY IfcPersonAndOrganization 
IFC4X3::IfcPersonAndOrganization::IfcPersonAndOrganization( int tag ) { m_tag = tag; }
void IFC4X3::IfcPersonAndOrganization::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPERSONANDORGANIZATION" << "(";
	if( m_ThePerson ) { stream << "#" << m_ThePerson->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_TheOrganization ) { stream << "#" << m_TheOrganization->m_tag; } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_Roles );
	stream << ");";
}
void IFC4X3::IfcPersonAndOrganization::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcPersonAndOrganization::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 3 ){ std::stringstream err; err << "Wrong parameter count for entity IfcPersonAndOrganization, expecting 3, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReference( args[0], m_ThePerson, map, errorStream );
	readEntityReference( args[1], m_TheOrganization, map, errorStream );
	readEntityReferenceList( args[2], m_Roles, map, errorStream );
}
void IFC4X3::IfcPersonAndOrganization::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	vec_attributes.emplace_back( std::make_pair( "ThePerson", m_ThePerson ) );
	vec_attributes.emplace_back( std::make_pair( "TheOrganization", m_TheOrganization ) );
	shared_ptr<AttributeObjectVector> Roles_vec_object( new AttributeObjectVector() );
	std::copy( m_Roles.begin(), m_Roles.end(), std::back_inserter( Roles_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "Roles", Roles_vec_object ) );
}
void IFC4X3::IfcPersonAndOrganization::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
}
void IFC4X3::IfcPersonAndOrganization::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	shared_ptr<IfcPersonAndOrganization> ptr_self = dynamic_pointer_cast<IfcPersonAndOrganization>( ptr_self_entity );
	if( !ptr_self ) { throw BuildingException( "IfcPersonAndOrganization::setInverseCounterparts: type mismatch" ); }
	if( m_TheOrganization )
	{
		m_TheOrganization->m_Engages_inverse.emplace_back( ptr_self );
	}
	if( m_ThePerson )
	{
		m_ThePerson->m_EngagedIn_inverse.emplace_back( ptr_self );
	}
}
void IFC4X3::IfcPersonAndOrganization::unlinkFromInverseCounterparts()
{
	if( m_TheOrganization )
	{
		std::vector<weak_ptr<IfcPersonAndOrganization> >& Engages_inverse = m_TheOrganization->m_Engages_inverse;
		for( auto it_Engages_inverse = Engages_inverse.begin(); it_Engages_inverse != Engages_inverse.end(); )
		{
			weak_ptr<IfcPersonAndOrganization> self_candidate_weak = *it_Engages_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_Engages_inverse;
				continue;
			}
			shared_ptr<IfcPersonAndOrganization> self_candidate( *it_Engages_inverse );
			if( self_candidate.get() == this )
			{
				it_Engages_inverse= Engages_inverse.erase( it_Engages_inverse );
			}
			else
			{
				++it_Engages_inverse;
			}
		}
	}
	if( m_ThePerson )
	{
		std::vector<weak_ptr<IfcPersonAndOrganization> >& EngagedIn_inverse = m_ThePerson->m_EngagedIn_inverse;
		for( auto it_EngagedIn_inverse = EngagedIn_inverse.begin(); it_EngagedIn_inverse != EngagedIn_inverse.end(); )
		{
			weak_ptr<IfcPersonAndOrganization> self_candidate_weak = *it_EngagedIn_inverse;
			if( self_candidate_weak.expired() )
			{
				++it_EngagedIn_inverse;
				continue;
			}
			shared_ptr<IfcPersonAndOrganization> self_candidate( *it_EngagedIn_inverse );
			if( self_candidate.get() == this )
			{
				it_EngagedIn_inverse= EngagedIn_inverse.erase( it_EngagedIn_inverse );
			}
			else
			{
				++it_EngagedIn_inverse;
			}
		}
	}
}
