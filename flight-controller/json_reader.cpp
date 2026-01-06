#include "json_reader.h"
#include "json_builder.h"

using namespace std;
using namespace json;
using namespace geo;
using namespace reader;

void reader::JsonReader::ApplyBaseRequest(const json::Node& node) {
    auto description = GetBusesAndStopDescription(node);

    for (const auto& stop_info : description.second) {
        tc_.AddStop(stop_info.name, stop_info.coord);
    }

    for (const auto& stop_info : description.second) {
        for (const auto& [stop_name, distance] : stop_info.stops_dist) {
            tc_.UpdateStopsDist(stop_info.name, { stop_name, distance });
        }
    }

    for (const auto& bus_info : description.first) {
        tc_.AddBus(bus_info.name, bus_info.stops, bus_info.is_roundtrip);
    }
}


void reader::JsonReader::ApplyStartRequest(std::ostringstream& render_out, const json::Node& node, ostream& out) {
    Builder builder;
    builder.StartArray();


    TransportRouter tr(tc_, node.AsDict().at("bus_velocity").AsInt(), node.AsDict().at("bus_wait_time").AsInt());


    for (const auto& array_item : node.AsArray()) {
        const auto& request_map = array_item.AsDict();
        auto request_id = request_map.at("id").AsInt();
        const string& type = request_map.at("type").AsString();

        if (type == "Bus") {
            const string& name = request_map.at("name").AsString();
            auto bi = tc_.GetBusInfo(name);
            builder.StartDict();
            builder.Key("request_id").Value(request_id);
            if (bi.was_found) {
                builder.Key("curvature").Value(bi.coef)
                    .Key("route_length").Value(bi.length)
                    .Key("stop_count").Value(bi.stops)
                    .Key("unique_stop_count").Value(bi.unic_stops);
            }
            else {
                builder.Key("error_message"s).Value("not found"s);
            }
            builder.EndDict();
        }
        else if (type == "Stop") {
            const string& name = request_map.at("name").AsString();
            auto si = tc_.GetStopInfo(name);
            builder.StartDict();
            builder.Key("request_id").Value(request_id);
            if (si.has_value() && si.value() != nullptr) {
                builder.Key("buses").StartArray();
                for (const auto& bus : *si.value()) {
                    builder.Value(bus);
                }
                builder.EndArray();
            }
            else if (si == nullptr) {
                builder.Key("buses").StartArray().EndArray();
            }
            else {
                builder.Key("error_message"s).Value("not found"s);
            }
            builder.EndDict();
        }
        else if (type == "Map") {
            builder.StartDict()
                .Key("request_id").Value(request_id)
                .Key("map").Value(render_out.str())
                .EndDict();
        }
        else if (type == "Route") {
            string_view from = array_item.AsDict().at("from").AsString();
            string_view to = array_item.AsDict().at("to").AsString();

            auto route_info = tr.CalculateRoute(from, to);
            if (!route_info) {
                builder.StartDict()
                    .Key("request_id").Value(request_id)
                    .Key("error_message").Value("not found"s)
                    .EndDict();
            }
            else {
                builder.StartDict()
                    .Key("request_id").Value(request_id)
                    .Key("total_time").Value(route_info->total_time)
                    .Key("items").StartArray();

                for (const auto& item : route_info->items) {
                    if (item.type == RouteItem::Type::Wait) {
                        builder.StartDict()
                            .Key("type").Value("Wait"s)
                            .Key("stop_name").Value(item.name)
                            .Key("time").Value(item.time)
                            .EndDict();
                    }
                    else {
                        builder.StartDict()
                            .Key("type").Value("Bus"s)
                            .Key("bus").Value(item.name)
                            .Key("span_count").Value(item.span_count)
                            .Key("time").Value(item.time)
                            .EndDict();
                    }
                }

                builder.EndArray().EndDict();
            }
        }
    }

    builder.EndArray();
    Node n = builder.Build();
    out << Print(n);
}

RenderSettings reader::JsonReader::GetRenderSettings(const json::Node& node) {
    RenderSettings res;

    if (!node.IsDict()) {
        throw std::logic_error("Json parse err");
    }
    const auto& node_map = node.AsDict();

    res.width = node_map.at("width").AsDouble();
    res.height = node_map.at("height").AsDouble();
    res.padding = node_map.at("padding").AsDouble();
    res.line_width = node_map.at("line_width").AsDouble();
    res.stop_radius = node_map.at("stop_radius").AsDouble();
    res.bus_label_font_size = node_map.at("bus_label_font_size").AsInt();

    const auto& bus_label_offset = node_map.at("bus_label_offset").AsArray();
    res.bus_label_offset_point.x = bus_label_offset[0].AsDouble();
    res.bus_label_offset_point.y = bus_label_offset[1].AsDouble();

    res.stop_label_font_size = node_map.at("stop_label_font_size").AsInt();

    const auto& stop_label_offset = node_map.at("stop_label_offset").AsArray();
    res.stop_label_offset_point.x = stop_label_offset[0].AsDouble();
    res.stop_label_offset_point.y = stop_label_offset[1].AsDouble();

    res.underlayer_color = ParseColor(node_map.at("underlayer_color"));
    res.underlayer_width = node_map.at("underlayer_width").AsDouble();

    for (const auto& color_node : node_map.at("color_palette").AsArray()) {
        res.color_palette.push_back(ParseColor(color_node));
    }

    return res;
}

pair<vector<BusDescription>, vector<StopDescription>> reader::JsonReader::GetBusesAndStopDescription(const json::Node& node) {
    vector<BusDescription> buses_info;
    vector<StopDescription> stops_info;

    for (const auto& array_item : node.AsArray()) {
        if (!array_item.IsDict()) {
            continue;
        }

        const auto& node_map = array_item.AsDict();
        const string& type = node_map.at("type").AsString();

        if (type == "Stop") {
            const string& name = node_map.at("name"s).AsString();
            double lat = node_map.at("latitude"s).AsDouble();
            double lon = node_map.at("longitude"s).AsDouble();
            Coordinates coord{ lat, lon };

            std::unordered_map<std::string, double> stops_dist;
            auto dist_it = node_map.find("road_distances");
            if (dist_it != node_map.end() && dist_it->second.IsDict()) {
                for (const auto& [stop_name, distance_node] : dist_it->second.AsDict()) {
                    stops_dist[stop_name] = distance_node.AsDouble();
                }
            }

            stops_info.emplace_back(name, coord, std::move(stops_dist));
        }
        else if (type == "Bus") {
            const auto& name = node_map.at("name"s).AsString();

            std::vector<std::string_view> stops;
            const auto& stops_array = node_map.at("stops"s).AsArray();
            stops.reserve(stops_array.size());
            for (const auto& stop_node : stops_array) {
                const auto& stop_name = stop_node.AsString();
                stops.emplace_back(stop_name);
            }
            bool is_roundtrip = node_map.at("is_roundtrip"s).AsBool();

            buses_info.emplace_back(name, std::move(stops), is_roundtrip);
        }
        else {
            throw "Unknown command\n";
        }
    }
    return std::pair<std::vector<BusDescription>, std::vector<StopDescription>>(buses_info, stops_info);
}

void reader::JsonReader::ApplyBusRequest(const string& name, int request_id, Builder& builder) {
    auto bi = tc_.GetBusInfo(name);
    if (bi.was_found) {
        builder.StartDict()
            .Key("request_id"s).Value(request_id)
            .Key("curvature"s).Value(bi.coef)
            .Key("route_length"s).Value(bi.length)
            .Key("stop_count"s).Value(bi.stops)
            .Key("unique_stop_count"s).Value(bi.unic_stops)
            .EndDict();
    }
    else {
        builder.StartDict()
            .Key("request_id"s).Value(request_id)
            .Key("error_message"s).Value("not found"s)
            .EndDict();
    }
}

void reader::JsonReader::ApplyStopRequest(const string& name, int request_id, Builder& builder) {
    auto si = tc_.GetStopInfo(name);
    if (si.has_value() && si.value() != nullptr) {
        if (si.value()->empty()) {
            builder.StartDict()
                .Key("request_id"s).Value(request_id)
                .Key("buses"s).StartArray().EndArray()
                .EndDict();
        }
        else {
            builder.StartDict()
                .Key("request_id"s).Value(request_id)
                .Key("buses"s).StartArray();
            for (auto it = si.value()->begin(); it != si.value()->end(); it++) {
                builder.Value(*it);
            }
            builder.EndArray().EndDict();
        }
    }
    else if (si == nullptr) {
        builder.StartDict()
            .Key("request_id"s).Value(request_id)
            .Key("buses"s).StartArray().EndArray()
            .EndDict();
    }
    else {
        builder.StartDict()
            .Key("request_id"s).Value(request_id)
            .Key("error_message"s).Value("not found"s)
            .EndDict();
    }
}

std::string reader::Print(const Node& node) {
    std::ostringstream out;
    json::Print(Document{ node }, out);
    return out.str();
}

svg::Color reader::ParseColor(const json::Node& color_node) {
    if (color_node.IsString()) {
        return color_node.AsString();
    }

    const auto& arr = color_node.AsArray();
    if (arr.size() == 3) {
        return svg::Rgb{
            static_cast<uint8_t>(arr[0].AsInt()),
            static_cast<uint8_t>(arr[1].AsInt()),
            static_cast<uint8_t>(arr[2].AsInt())
        };
    }
    else if (arr.size() == 4) {
        return svg::Rgba{
            static_cast<uint8_t>(arr[0].AsInt()),
            static_cast<uint8_t>(arr[1].AsInt()),
            static_cast<uint8_t>(arr[2].AsInt()),
            arr[3].AsDouble()
        };
    }

    return svg::NoneColor;
}