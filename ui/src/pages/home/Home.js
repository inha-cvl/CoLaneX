import React from "react";
import logo from "../../logo.png";
import "./Home.css";

const Home = () => {
  return (
    <div className="Home">
      <header className="Home-header">
        <img src={logo} className="Home-logo" alt="logo" />
        <p> Cooperative Lane Change using by V2X </p>
      </header>
    </div>
  );
};

export default Home;