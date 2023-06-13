import React from 'react'
import {Nav, NavSpan, NavLink1, NavLink2} from "./NavbarElements"

const Navbar = () => {
    return (
        <>
          <Nav>
              <NavSpan>
                <NavLink1 to="/hlv" activeStyle>
                    HLV
                </NavLink1>
                <NavLink2 to="/tlv" activeStyle>
                    TLV
                </NavLink2>
              </NavSpan>
          </Nav>  
        </>
    );
};

export default Navbar;