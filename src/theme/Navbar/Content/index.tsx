/**
 * Custom Navbar Content with Better Auth Integration
 * Adds Sign In/Sign Up/Logout buttons based on authentication state
 */

import React from 'react';
import { useThemeConfig, ErrorCauseBoundary } from '@docusaurus/theme-common';
import {
  splitNavbarItems,
  useNavbarMobileSidebar,
} from '@docusaurus/theme-common/internal';
import NavbarItem, { type Props as NavbarItemConfig } from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import SearchBar from '@theme/SearchBar';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarSearch from '@theme/Navbar/Search';
import Link from '@docusaurus/Link';
import { useSession } from '../../../lib/auth-client';
import styles from './styles.module.css';

function useNavbarItems() {
  // TODO temporary casting until ThemeConfig type is improved
  return useThemeConfig().navbar.items as NavbarItemConfig[];
}

function NavbarItems({ items }: { items: NavbarItemConfig[] }): JSX.Element {
  return (
    <>
      {items.map((item, i) => (
        <ErrorCauseBoundary
          key={i}
          onError={(error) =>
            new Error(
              `A theme navbar item failed to render.
Please double-check the following navbar item (themeConfig.navbar.items) of your Docusaurus config:
${JSON.stringify(item, null, 2)}`,
              { cause: error },
            )
          }>
          <NavbarItem {...item} />
        </ErrorCauseBoundary>
      ))}
    </>
  );
}

function NavbarContentLayout({
  left,
  right,
}: {
  left: React.ReactNode;
  right: React.ReactNode;
}) {
  return (
    <div className="navbar__inner">
      <div className="navbar__items">{left}</div>
      <div className="navbar__items navbar__items--right">{right}</div>
    </div>
  );
}

function AuthButtons() {
  const { data: session, isPending } = useSession();

  if (isPending) {
    return (
      <div className={styles.authButtons}>
        <span className={styles.authLoading}>Loading...</span>
      </div>
    );
  }

  if (session?.user) {
    return (
      <div className={styles.authButtons}>
        <span className={styles.userEmail}>
          {session.user.name || session.user.email}
        </span>
        <button
          onClick={async () => {
            const { authClient } = await import('../../../lib/auth-client');
            await authClient.signOut();
            window.location.href = '/';
          }}
          className={styles.authButton}>
          Logout
        </button>
      </div>
    );
  }

  return (
    <div className={styles.authButtons}>
      <Link to="/auth/signin" className={styles.authLink}>
        Sign In
      </Link>
      <Link to="/auth/signup" className={styles.authButtonPrimary}>
        Sign Up
      </Link>
    </div>
  );
}

export default function NavbarContent(): JSX.Element {
  const mobileSidebar = useNavbarMobileSidebar();

  const items = useNavbarItems();
  const [leftItems, rightItems] = splitNavbarItems(items);

  const searchBarItem = items.find((item) => item.type === 'search');

  return (
    <NavbarContentLayout
      left={
        // TODO stop hardcoding items?
        <>
          {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
          <NavbarLogo />
          <NavbarItems items={leftItems} />
        </>
      }
      right={
        // TODO stop hardcoding items?
        // Ask the user to add the respective navbar items => more flexible
        <>
          <NavbarItems items={rightItems} />
          <NavbarColorModeToggle className="navbar__item" />
          {!searchBarItem && (
            <NavbarSearch>
              <SearchBar />
            </NavbarSearch>
          )}
          <AuthButtons />
        </>
      }
    />
  );
}
