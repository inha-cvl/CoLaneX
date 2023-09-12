package com.obigo.v2x.controller;

import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestMethod;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.servlet.ModelAndView;

import java.util.Stack;

@Controller
@Slf4j
public class IndexController {


    @RequestMapping(value = "/", method = RequestMethod.GET)
    public String index() {
        return "redirect:front";
    }

    @RequestMapping(value = "/front", method = RequestMethod.GET)
    public ModelAndView front(Model m) {
        log.debug("Map page start...");

        ModelAndView mv = new ModelAndView();

        mv.setViewName("/front/front");
        return mv;
    }

    @RequestMapping(value = "/front2", method = RequestMethod.GET)
    public ModelAndView front2(Model m) {
        log.debug("Map page start...");

        ModelAndView mv = new ModelAndView();

        mv.setViewName("/front/front2");

        return mv;
    }
}
