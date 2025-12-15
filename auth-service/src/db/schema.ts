import { pgTable, text, timestamp, boolean } from 'drizzle-orm/pg-core';

// Experience Level Enum (as text with constraint)
export const experienceLevels = ['beginner', 'intermediate', 'advanced'] as const;
export type ExperienceLevel = typeof experienceLevels[number];

// Professional Role Enum (as text with constraint)
export const professionalRoles = ['student', 'researcher', 'engineer', 'hobbyist', 'other'] as const;
export type ProfessionalRole = typeof professionalRoles[number];

// User table (Better Auth core + custom fields)
export const user = pgTable('user', {
  id: text('id').primaryKey(),
  email: text('email').notNull().unique(),
  emailVerified: boolean('emailVerified').notNull().default(false),
  name: text('name'),
  createdAt: timestamp('createdAt').notNull(),
  updatedAt: timestamp('updatedAt').notNull(),
  image: text('image'),

  // Custom fields for user background
  experienceLevel: text('experience_level').notNull().$type<ExperienceLevel>(),
  professionalRole: text('professional_role').notNull().$type<ProfessionalRole>(),
  roleOther: text('role_other'),
  organization: text('organization'),
});

// Session table (Better Auth)
export const session = pgTable('session', {
  id: text('id').primaryKey(),
  expiresAt: timestamp('expiresAt').notNull(),
  ipAddress: text('ipAddress'),
  userAgent: text('userAgent'),
  userId: text('userId')
    .notNull()
    .references(() => user.id, { onDelete: 'cascade' }),
  activeOrganizationId: text('activeOrganizationId'),
  impersonatedBy: text('impersonatedBy'),
});

// Account table (Better Auth - for OAuth providers)
export const account = pgTable('account', {
  id: text('id').primaryKey(),
  accountId: text('accountId').notNull(),
  providerId: text('providerId').notNull(),
  userId: text('userId')
    .notNull()
    .references(() => user.id, { onDelete: 'cascade' }),
  accessToken: text('accessToken'),
  refreshToken: text('refreshToken'),
  idToken: text('idToken'),
  expiresAt: timestamp('expiresAt'),
  password: text('password'),
});

// Verification table (Better Auth - for email verification)
export const verification = pgTable('verification', {
  id: text('id').primaryKey(),
  identifier: text('identifier').notNull(),
  value: text('value').notNull(),
  expiresAt: timestamp('expiresAt').notNull(),
  createdAt: timestamp('createdAt'),
  updatedAt: timestamp('updatedAt'),
});

// Export types
export type User = typeof user.$inferSelect;
export type NewUser = typeof user.$inferInsert;

export type Session = typeof session.$inferSelect;
export type NewSession = typeof session.$inferInsert;

export type Account = typeof account.$inferSelect;
export type NewAccount = typeof account.$inferInsert;

export type Verification = typeof verification.$inferSelect;
export type NewVerification = typeof verification.$inferInsert;
