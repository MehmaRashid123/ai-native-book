/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React from 'react';
import NavbarLayout from '@theme/Navbar/Layout';
import Link from '@docusaurus/Link';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import SearchBar from '@theme/SearchBar';
import {useNavbarMobileSidebar} from '@docusaurus/theme-common/internal';
import clsx from 'clsx';
import { useAuth } from '../../components/Auth/AuthContext';
import styles from './styles.module.css'; // Import local styles

export default function Navbar() {
  const mobileSidebar = useNavbarMobileSidebar();
  const { user, isLoading } = useAuth();

  return (
    <NavbarLayout>
      <div className={clsx(styles.navbarSection, styles.navbarSectionLeft)}>
        {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
        <NavbarLogo />
        <div className={styles.navbarTitle}>Physical AI</div>
      </div>

      <div className={clsx(styles.navbarSection, styles.navbarSectionCenter)}>
        <Link className={clsx(styles.navbarLink, styles.navbarItem)} to="/docs/module-1-ros2/introduction">Textbook</Link>
        <Link className={clsx(styles.navbarLink, styles.navbarItem)} to="/labs">Labs</Link>
        <Link className={clsx(styles.navbarLink, styles.navbarItem)} to="/capstone">Capstone</Link>
        <Link className={clsx(styles.navbarLink, styles.navbarItem)} to="/reference">Reference</Link>
        {user && <Link className={clsx(styles.navbarLink, styles.navbarItem)} to="/profile">Profile</Link>}
      </div>

      <div className={clsx(styles.navbarSection, styles.navbarSectionRight)}>
        {!user ? (
          <>
            <Link className={clsx(styles.button, styles.buttonSecondary)} to="/login">Login</Link>
            <Link className={clsx(styles.button, styles.buttonPrimary)} to="/signup">Signup</Link>
          </>
        ) : (
          <div className={styles.navbarTitle} style={{fontSize: '0.8rem', color: 'var(--neon-cyan)'}}>Hello, {user.name.split(' ')[0]}</div>
        )}
        <div className={clsx(styles.navbarLink, styles.navbarItem, styles.navbarLinkRight)}>اردو</div>
        <NavbarColorModeToggle className={styles.colorModeToggle} />
        <SearchBar />
      </div>
    </NavbarLayout>
  );
}
