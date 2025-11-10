package com.cmds.webapp.controllers;

import org.springframework.stereotype.Controller;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.ui.Model;

@Controller
public class DashboardController {
    @GetMapping("/dashboard")
    public String dashboard(Model model) {
        return "dashboard";
    }
}