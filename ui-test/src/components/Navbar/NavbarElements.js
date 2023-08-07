import styled from "styled-components"
import {NavLink as Link} from "react-router-dom"
import {FaBars} from "react-icons/fa"

export const Nav = styled.nav`
    margin-top: 0;
    background: linear-gradient(135deg, #362577, #85dcf5, #274f8a);
    justify-content: space-around;
    padding-bottom: 4px;
`;

export const NavSpan = styled.nav`
    margin-top: 0;
    background: #0c1024;
    height: 80px;
    display: flex;
    justify-content: space-around;

    padding: 0.5rem ;
    z-index: 10;   
`;

export const NavLink1 = styled(Link)`
    margin-top: 0.5rem;
    color: #fff;
    display: flex;
    align-items: center;
    font-size: 5rem;
    text-decoration: none;
    padding: 0 1rem;
    height: 100%;
    cursor: pointer;    
    &.active{
        color: #ff75ba;
    }
    &:hover{
        transition: all 0.2s ease-in-out;
        color: #ff75ba;
    }
`;
export const NavLink2 = styled(Link)`
    margin-top: 0.5rem;
    color: #fff;
    display: flex;
    align-items: center;
    font-size: 5rem;
    text-decoration: none;
    padding: 0 1rem;
    height: 100%;
    cursor: pointer;    
    &.active{
        color: #5cdcff;
    }
    &:hover{
        transition: all 0.2s ease-in-out;
        color: #5cdcff;
    }
`;