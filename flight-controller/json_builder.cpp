#include "json_builder.h"

using namespace json;

BaseContext::BaseContext(Builder& builder) : builder_(builder) {}

Builder::Builder() {
    nodes_stack_.reserve(8);
}

void Builder::CanStartDict() const {
    if (state_ != BuilderState::Empty && state_ != BuilderState::AfterKey && state_ != BuilderState::Array) {
        throw std::logic_error("StartDict() err");
    }
}

void Builder::CanStartArray() const {
    if (state_ != BuilderState::Empty && state_ != BuilderState::AfterKey && state_ != BuilderState::Array) {
        throw std::logic_error("StartArray() err");
    }
}

void Builder::CanKey() const {
    if (state_ != BuilderState::Dict && state_ != BuilderState::AfterDictValue) {
        throw std::logic_error("Key() err");
    }
}

void Builder::CanValue() const {
    if (state_ != BuilderState::Empty && state_ != BuilderState::AfterKey && state_ != BuilderState::Array) {
        throw std::logic_error("Value() err");
    }
}

void Builder::CanEndDict() const {
    if (state_ != BuilderState::Dict && state_ != BuilderState::AfterDictValue) {
        throw std::logic_error("EndDict() err");
    }
}

void Builder::CanEndArray() const {
    if (state_ != BuilderState::Array && state_ != BuilderState::AfterArrayValue) {
        throw std::logic_error("EndArray() err");
    }
}

DictItemContext Builder::StartDict() {
    CanStartDict();

    if (!nodes_stack_.empty() && nodes_stack_.back()->IsArray()) {
        Array& array = const_cast<Array&>(nodes_stack_.back()->AsArray());
        array.emplace_back(Dict{});
        nodes_stack_.push_back(&array.back());
    }
    else if (!keys_.empty()) {
        Dict& dict = const_cast<Dict&>(nodes_stack_.back()->AsDict());
        dict[keys_.back()] = Dict{};
        nodes_stack_.push_back(&dict[keys_.back()]);
        keys_.pop_back();
    }
    else if (nodes_stack_.empty() && root_.IsNull()) {
        root_ = Dict{};
        nodes_stack_.push_back(&root_);
    }
    else {
        throw std::logic_error("StartDict() err");
    }

    state_ = BuilderState::Dict;
    return DictItemContext(*this);
}

ArrayItemContext Builder::StartArray() {
    CanStartArray();

    if (!nodes_stack_.empty() && nodes_stack_.back()->IsArray()) {
        Array& array = const_cast<Array&>(nodes_stack_.back()->AsArray());
        array.emplace_back(Array{});
        nodes_stack_.push_back(&array.back());
    }
    else if (!keys_.empty()) {
        Dict& dict = const_cast<Dict&>(nodes_stack_.back()->AsDict());
        dict[keys_.back()] = Array{};
        nodes_stack_.push_back(&dict[keys_.back()]);
        keys_.pop_back();
    }
    else if (nodes_stack_.empty() && root_.IsNull()) {
        root_ = Array{};
        nodes_stack_.push_back(&root_);
    }
    else {
        throw std::logic_error("StartArray() err");
    }

    state_ = BuilderState::Array;
    return ArrayItemContext(*this);
}

Builder& Builder::EndDict() {
    CanEndDict();

    if (nodes_stack_.empty() || !nodes_stack_.back()->IsDict()) {
        throw std::logic_error("EndDict() err");
    }

    nodes_stack_.pop_back();
    state_ = nodes_stack_.empty() ? BuilderState::Empty :
        (nodes_stack_.back()->IsArray() ? BuilderState::Array : BuilderState::Dict);
    return *this;
}

Builder& Builder::EndArray() {
    CanEndArray();

    if (nodes_stack_.empty() || !nodes_stack_.back()->IsArray()) {
        throw std::logic_error("EndArray() err");
    }

    nodes_stack_.pop_back();
    state_ = nodes_stack_.empty() ? BuilderState::Empty :
        (nodes_stack_.back()->IsArray() ? BuilderState::Array : BuilderState::Dict);
    return *this;
}

KeyContext Builder::Key(std::string key) {
    CanKey();

    if (nodes_stack_.empty() || !nodes_stack_.back()->IsDict()) {
        throw std::logic_error("Key() err");
    }

    keys_.push_back(std::move(key));
    state_ = BuilderState::AfterKey;
    return KeyContext(*this);
}

Builder& Builder::Value(Node value) {
    CanValue();

    if (!nodes_stack_.empty() && nodes_stack_.back()->IsArray()) {
        Array& array = const_cast<Array&>(nodes_stack_.back()->AsArray());
        array.emplace_back(std::move(value));
        state_ = BuilderState::Array;
    }
    else if (!keys_.empty()) {
        Dict& dict = const_cast<Dict&>(nodes_stack_.back()->AsDict());
        dict[keys_.back()] = std::move(value);
        keys_.pop_back();
        state_ = BuilderState::AfterDictValue;
    }
    else if (nodes_stack_.empty() && root_.IsNull()) {
        root_ = std::move(value);
        state_ = BuilderState::Empty;
    }
    else {
        throw std::logic_error("Value() err");
    }

    return *this;
}

Node Builder::Build() {
    if (!nodes_stack_.empty()) {
        throw std::logic_error("Build() err");
    }
    if (root_.IsNull()) {
        throw std::logic_error("Build() err");
    }
    return std::move(root_);
}

KeyContext DictItemContext::Key(std::string key) {
    return builder_.Key(std::move(key));
}

Builder& DictItemContext::EndDict() {
    return builder_.EndDict();
}

ArrayItemContext ArrayItemContext::Value(Node value) {
    builder_.Value(std::move(value));
    return *this;
}

DictItemContext ArrayItemContext::StartDict() {
    return builder_.StartDict();
}

ArrayItemContext ArrayItemContext::StartArray() {
    return builder_.StartArray();
}

Builder& ArrayItemContext::EndArray() {
    return builder_.EndArray();
}

DictItemContext KeyContext::Value(Node value) {
    builder_.Value(std::move(value));
    return DictItemContext(builder_);
}

ArrayItemContext KeyContext::StartArray() {
    return builder_.StartArray();
}

DictItemContext KeyContext::StartDict() {
    return builder_.StartDict();
}