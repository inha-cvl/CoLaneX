import React from "react";
import { Route, BrowserRouter, Routes } from "react-router-dom";
import { ThemeProvider } from "@material-ui/core";
import Navbar from "./components/Navbar";
import mainTheme from "./mainTheme";
import Home from "./pages/home/Home";
import HLV from "./pages/hlv/HLV";
import TLV from "./pages/tlv/TLV";

function App() {
  return (
    <ThemeProvider theme={mainTheme}>
      <div>
        <BrowserRouter>
          <Navbar />
          <Routes>
            <Route path="/" element={<Home />}/>
            <Route path="/hlv" element={<HLV />} />
            <Route path="/tlv" element={<TLV />} />
          </Routes>
        </BrowserRouter>
      </div>
    </ThemeProvider>
  );
}

export default App;