#pragma once

#include "json.h"
#include "domain.h"
#include "transport_catalogue.h"
#include "request_handler.h"
#include "json_builder.h"

#include <string>
#include <iostream>  
#include <sstream>

/*
 * Здесь можно разместить код наполнения транспортного справочника данными из JSON,
 * а также код обработки запросов к базе и формирование массива ответов в формате JSON
 */
namespace reader {

    class JsonReader {
    public:
        JsonReader() = default;
        JsonReader(transport::TransportCatalogue& tc) : tc_(tc) {};

        void ApplyBaseRequest(const json::Node& node);
        void ApplyStartRequest(std::ostringstream& render_out, const json::Node& node, std::ostream& out);
        RenderSettings GetRenderSettings(const json::Node& node);

    private:
        transport::TransportCatalogue& tc_;

        std::pair<std::vector<BusDescription>, std::vector<StopDescription>> GetBusesAndStopDescription(const json::Node& node);
        void ApplyBusRequest(const std::string& name, int request_id, json::Builder& builder);
        void ApplyStopRequest(const std::string& name, int request_id, json::Builder& builder);
    };

    std::string Print(const json::Node& node);
    svg::Color ParseColor(const json::Node& color_node);
}
