/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React from 'react';
import NavbarLayout from '@theme/Navbar/Layout';
import NavbarContent from '@theme/Navbar/Content';
import Link from '@docusaurus/Link';

export default function Navbar() {
  return (
    <NavbarLayout>
      <div className="navbar__title">Physical AI</div>
      <div className="navbar__items navbar__items--center">
        <Link className="navbar__item navbar__link" to="/docs/intro">Textbook</Link>
        <Link className="navbar__item navbar__link" to="/labs">Labs</Link>
        <Link className="navbar__item navbar__link" to="/capstone">Capstone</Link>
        <Link className="navbar__item navbar__link" to="/reference">Reference</Link>
      </div>
      <NavbarContent />
    </NavbarLayout>
  );
}
