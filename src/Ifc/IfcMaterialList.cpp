/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcMaterial.h"
#include "ifcpp/Ifc/IfcMaterialList.h"

// ENTITY IfcMaterialList 
IFC4X3::IfcMaterialList::IfcMaterialList( int tag ) { m_tag = tag; }
void IFC4X3::IfcMaterialList::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCMATERIALLIST" << "(";
	writeEntityList( stream, m_Materials );
	stream << ");";
}
void IFC4X3::IfcMaterialList::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcMaterialList::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 1 ){ std::stringstream err; err << "Wrong parameter count for entity IfcMaterialList, expecting 1, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	readEntityReferenceList( args[0], m_Materials, map, errorStream );
}
void IFC4X3::IfcMaterialList::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	shared_ptr<AttributeObjectVector> Materials_vec_object( new AttributeObjectVector() );
	std::copy( m_Materials.begin(), m_Materials.end(), std::back_inserter( Materials_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "Materials", Materials_vec_object ) );
}
void IFC4X3::IfcMaterialList::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
}
void IFC4X3::IfcMaterialList::setInverseCounterparts( shared_ptr<BuildingEntity> )
{
}
void IFC4X3::IfcMaterialList::unlinkFromInverseCounterparts()
{
}
