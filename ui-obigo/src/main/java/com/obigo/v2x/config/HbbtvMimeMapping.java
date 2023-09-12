package com.obigo.v2x.config;

import org.springframework.boot.web.server.MimeMappings;
import org.springframework.boot.web.server.WebServerFactoryCustomizer;
import org.springframework.boot.web.servlet.server.ConfigurableServletWebServerFactory;
import org.springframework.context.annotation.Configuration;

@Configuration
public class HbbtvMimeMapping implements WebServerFactoryCustomizer<ConfigurableServletWebServerFactory> {
    @Override
    public void customize(ConfigurableServletWebServerFactory factory) {
        MimeMappings mappings = new MimeMappings(MimeMappings.DEFAULT);
//        mappings.add("mtl", "text/html,application/xhtml+xml,application/xml");
        mappings.add("mtl", "text/plain; charset=utf-8");
        mappings.add("obj", "text/plain; charset=utf-8");
//        mappings.add("tga", "text/plain; charset=utf-8");
//        mappings.add("model/mtl", "mtl");
//        mappings.add("xhtml", "application/vnd.hbbtv.xhtml+xml; charset=utf-8");
        factory.setMimeMappings(mappings);
    }
}