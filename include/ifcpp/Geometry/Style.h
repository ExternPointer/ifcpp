#pragma once
#include <memory>

namespace IFC4X3 {
class IfcRepresentationItem;
}

namespace ifcpp {
unsigned int convertRepresentationStyle( const std::shared_ptr<IFC4X3::IfcRepresentationItem>& representation_item );
}