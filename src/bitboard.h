/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2008 Tord Romstad (Glaurung author)
  Copyright (C) 2008-2015 Marco Costalba, Joona Kiiski, Tord Romstad
  Copyright (C) 2015-2018 Marco Costalba, Joona Kiiski, Gary Linscott, Tord Romstad

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BITBOARD_H_INCLUDED
#define BITBOARD_H_INCLUDED

#include <string>

#include "types.h"

namespace Bitbases {

void init();
  bool probe(Square wksq, Square wpsq, Square bksq, Color us);
}

namespace Bitboards {
  void init();
  const std::string pretty(Bitboard b);
  const std::string printBB(Bitboard b);
  const std::string dump();
}

// most significant bit is H8, and least significant bit is a1
//                                                    //  H8                                                                     A1
constexpr Bitboard AllSquares = ~Bitboard(0);           // 11111111 11111111 11111111 11111111 11111111 11111111 11111111 11111111
constexpr Bitboard DarkSquares = 0xAA55AA55AA55AA55ULL; // 10101010 01010101 10101010 01010101 10101010 01010101 10101010 01010101

// Using left shift to determine files from FILE_A
constexpr Bitboard FileABB = 0x0101010101010101ULL;     // 00000001 00000001 00000001 00000001 00000001 00000001 00000001 00000001
constexpr Bitboard FileBBB = FileABB << 1;              // 00000010 00000010 00000010 00000010 00000010 00000010 00000010 00000010
constexpr Bitboard FileCBB = FileABB << 2;              // 00000100 00000100 00000100 00000100 00000100 00000100 00000100 00000100
constexpr Bitboard FileDBB = FileABB << 3;              // 00001000 00001000 00001000 00001000 00001000 00001000 00001000 00001000
constexpr Bitboard FileEBB = FileABB << 4;              // 00010000 00010000 00010000 00010000 00010000 00010000 00010000 00010000
constexpr Bitboard FileFBB = FileABB << 5;              // 00100000 00100000 00100000 00100000 00100000 00100000 00100000 00100000
constexpr Bitboard FileGBB = FileABB << 6;              // 01000000 01000000 01000000 01000000 01000000 01000000 01000000 01000000
constexpr Bitboard FileHBB = FileABB << 7;              // 10000000 10000000 10000000 10000000 10000000 10000000 10000000 10000000

// Shifting 8 bits to the left changes the rank.
constexpr Bitboard Rank1BB = 0xFF;                      // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 11111111
constexpr Bitboard Rank2BB = Rank1BB << (8 * 1);        // 00000000 00000000 00000000 00000000 00000000 00000000 11111111 00000000
constexpr Bitboard Rank3BB = Rank1BB << (8 * 2);        // 00000000 00000000 00000000 00000000 00000000 11111111 00000000 00000000
constexpr Bitboard Rank4BB = Rank1BB << (8 * 3);        // 00000000 00000000 00000000 00000000 11111111 00000000 00000000 00000000
constexpr Bitboard Rank5BB = Rank1BB << (8 * 4);        // 00000000 00000000 00000000 11111111 00000000 00000000 00000000 00000000
constexpr Bitboard Rank6BB = Rank1BB << (8 * 5);        // 00000000 00000000 11111111 00000000 00000000 00000000 00000000 00000000
constexpr Bitboard Rank7BB = Rank1BB << (8 * 6);        // 00000000 11111111 00000000 00000000 00000000 00000000 00000000 00000000
constexpr Bitboard Rank8BB = Rank1BB << (8 * 7);        // 11111111 00000000 00000000 00000000 00000000 00000000 00000000 00000000

extern int SquareDistance[SQUARE_NB][SQUARE_NB];        // SquareDistance[64][64]
                                                        // SquareDistance[SQ_A1][SQ_A2] = SquareDistance[0][8]  = 1
                                                        // SquareDistance[SQ_A1][SQ_B2] = SquareDistance[0][9]  = 1
                                                        // SquareDistance[SQ_A1][SQ_C2] = SquareDistance[0][10] = 2
                                                        // SquareDistance[SQ_A1][SQ_D3] = SquareDistance[0][19] = 3
                                                        // SquareDistance[SQ_A1][SQ_H1] = SquareDistance[0][7]  = 7
                                                        // SquareDistance[SQ_A1][SQ_H1] = SquareDistance[0][7]  = 7

extern Bitboard SquareBB[SQUARE_NB];                    // SquareBB[64]
                                                        //SquareBB[SQ_A1] = SquareBB[0] =
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000001
                                                        // 1UL left shifted SQ_A1 (zero) times (no shifting)

                                                        // SquareBB[SQ_B1] = SquareBB[1]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000010
                                                        // 1UL left shifted SQ_B1 (one) times
// FileBB[FILE_A] = FileABB
extern Bitboard FileBB[FILE_NB];                        // FileBB[8]
                                                        // FileBB[FILE_A] = FileBB[0]
                                                        // 00000001 00000001 00000001 00000001 00000001 00000001 00000001 00000001

                                                        // FileBB[FILE_B] = FileBB[1]
                                                        //00000010 00000010 00000010 00000010 00000010 00000010 00000010 00000010

// RankBB[RANK_1] = Rank1BB
extern Bitboard RankBB[RANK_NB];                        // RankBB[8]
                                                        // RankBB[RANK_1] = RankBB[0]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 11111111

                                                        // RankBB[RANK_2] = RankBB[1]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 11111111 00000000

extern Bitboard AdjacentFilesBB[FILE_NB];               // AdjacentFilesBB[8];
                                                        // AdjacentFilesBB[FILE_A] = AdjacentFilesBB[0]
                                                        // 00000010 00000010 00000010 00000010 00000010 00000010 00000010 00000010
                                                        // (Adjacent file of file A is only File B)

                                                        // AdjacentFilesBB[FILE_B] = AdjacentFilesBB[1]
                                                        // 00000101 00000101 00000101 00000101 00000101 00000101 00000101 00000101
                                                        // (Adjacent files of file B are Files A and C)

extern Bitboard ForwardRanksBB[COLOR_NB][RANK_NB];      // ForwardRanksBB[2][8]
                                                        // ForwardRanks[WHITE][RANK_1] = ForwardRanks[0][0]
                                                        // 11111111 11111111 11111111 11111111 11111111 11111111 11111111 00000000

                                                        // ForwardRanks[WHITE][RANK_2] = ForwardRanks[0][1]
                                                        // 11111111 11111111 11111111 11111111 11111111 11111111 00000000 00000000

// Rank is always from WHITE's perspective
                                                        // ForwardRanks[BLACK][RANK_8] = ForwardRanks[1][7]
                                                        // 00000000 11111111 11111111 11111111 11111111 11111111 11111111 11111111

                                                        // ForwardRanksBB[BLACK][RANK_7] = ForwardRanks[1][6]
                                                        // 00000000 00000000 11111111 11111111 11111111 11111111 11111111 11111111

// Bitboard of squares between two squares.
extern Bitboard BetweenBB[SQUARE_NB][SQUARE_NB];        // BetweenBB[64][64]
                                                        // BetweenBB[SQ_A1][SQ_H1] = BetweenBB[0][7]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 01111110

                                                        // BetweenBB[SQ_A1][SQ_H8] = BetweenBB[0][63]
                                                        // 00000000 01000000 00100000 00010000 00001000 00000100 00000010 00000000

                                                        // BetweenBB[SQ_A1][SQ_A2] = BetweenBB[0][8]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
                                                        // No squares between a1 and a2

// If both squares are on the same line (rank, file, diagonal) then there is a bitboard that contains all squares in those lines.
// If they are not on the same line, the bitboard is zero.
extern Bitboard LineBB[SQUARE_NB][SQUARE_NB];           // LineBB[64][64]

                                                        // LineBB[SQ_A1][SQ_H1] = LineBB[0][7]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 11111111

                                                        // LineBB[SQ_A1][SQ_H8] = LineBB[0][63]
                                                        // 10000000 01000000 00100000 00010000 00001000 00000100 00000010 00000001

                                                        // LineBB[SQ_A1][SQ_A2] = LineBB[0][8]
                                                        // 00000001 00000001 00000001 00000001 00000001 00000001 00000001 00000001

                                                        // LineBB[SQ_A1][SQ_C2] = LineBB[0][10]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000
                                                        // a1 and c2 are not on same file, or rank

// Starting from the square, bitboard of distance squares
// [ ][0] is always 0x0
// [X][1] is the bitboard of squares that are around the square X
//
extern Bitboard DistanceRingBB[SQUARE_NB][8];           // DistanceRingBB[64][8]
                                                        // DistanceRingBB[SQ_A1][0] - all zeros

                                                        // DistanceRingBB[SQ_A1][1] - a2,b2,b1
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000011 00000010

                                                        // DistanceRingBB[SQ_A1][2] - a3,b3,c3,c2,c1
                                                        // 00000000 00000000 00000000 00000000 00000000 00000111 00000100 00000100

                                                        // DistanceRingBB[SQ_E4][0] - all zeros

                                                        // DistanceRingBB[SQ_E4][1] - d5,e5,f5,d4,f4,d4,e3,f3
                                                        // 00000000 00000000 00000000 00111000 00101000 00111000 00000000 00000000

// Forward squares on same file, starting from SQUARE, using the direction of COLOR
extern Bitboard ForwardFileBB[COLOR_NB][SQUARE_NB];     // ForwardFileBB[2][64]

                                                        // ForwardFileBB[WHITE][SQ_A1] - a2,a3,a4,a5,a6,a6,a8
                                                        // 00000001 00000001 00000001 00000001 00000001 00000001 00000001 00000000

                                                        // ForwardFileBB[WHITE][SQ_A2] - a3,a4,a5,a6,a7,a8
                                                        // 00000001 00000001 00000001 00000001 00000001 00000001 00000000 00000000

                                                        // ForwardFileBB[BLACK][SQ_H8] - h7, h6, h5, h4, h3, h2, h1
                                                        // 00000000 10000000 10000000 10000000 10000000 10000000 10000000 10000000


// Forward squares that are on the same file, and adjacent files, starting from SQUARE, and using the direction of COLOR
extern Bitboard PassedPawnMask[COLOR_NB][SQUARE_NB];    // PassedPawnMask[2][64]

                                                        // PassedPawnMask[WHITE][SQ_B1] - a2,a3,a4,a5,a6,a7,a8
                                                        //                                b2,b3,b4,b5,b6,b7,b8
                                                        //                                c2,c3,c4,c5,c6,c7,c8
                                                        // 00000111 00000111 00000111 00000111 00000111 00000111 00000111 00000000

                                                        // PassedPawnMask[WHITE][SQ_B2] - a3,a4,a5,a6,a7,a8
                                                        //                                b3,b4,b5,b6,b7,b8
                                                        //                                c3,c4,c5,c6,c7,c8
                                                        // 00000111 00000111 00000111 00000111 00000111 00000111 00000000 00000000

                                                        // PassedPawnMask[BLACK][SQ_B2] - a1, b1, c1
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000111

// Forward squares only on adjacent files, starting from the SQUARE, using the direction of COLOR
extern Bitboard PawnAttackSpan[COLOR_NB][SQUARE_NB];    // PawnAttackSpan[2][64]

                                                        // PawnAttackSpan[WHITE][SQ_B1]   a2,a3,a4,a5,a6,a7,a8
                                                        //                                c2,c3,c4,c5,c6,c7,c8
                                                        // 00000101 00000101 00000101 00000101 00000101 00000101 00000101 00000000

                                                        // PawnAttackSpan[WHITE][SQ_B2]   a3,a4,a5,a6,a7,a8
                                                        //                                c3,c4,c5,c6,c7,c8
                                                        // 00000101 00000101 00000101 00000101 00000101 00000101 00000000 00000000

                                                        // PassedPawnMask[BLACK][SQ_B2] - a1, c1
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000101

// Bitboard of squares that the piece can attack from the SQUARE
// Only for pieces: KING, QUEEN, ROOK, BISHOP, and KNIGHT
extern Bitboard PseudoAttacks[PIECE_TYPE_NB][SQUARE_NB];// PseudoAttacks[8][64]

                                                        // PseudoAttacks[BISHOP][SQ_A1]
                                                        // 10000000 01000000 00100000 00010000 00001000 00000100 00000010 00000000

                                                        // PseudoAttacks[ROOK][SQ_A1]
                                                        // 00000001 00000001 00000001 00000001 00000001 00000001 00000001 11111110

                                                        // PseudoAttacks[QUEEN][SQ_A1]
                                                        // 10000001 01000001 00100001 00010001 00001001 00000101 00000011 11111110

                                                        // PseudoAttacks[KING][SQ_A1]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000011 00000010

                                                        // PseudoAttacks[KNIGHT][SQ_A1]
                                                        // 00000000 00000000 00000000 00000000 00000000 00000010 00000100 00000000

// Bitboard of squares that a pawn on SQUARE can attack, using the direction of the COLOR
extern Bitboard PawnAttacks[COLOR_NB][SQUARE_NB];       // PawnAttacks[2[64]

                                                        // PawnAttacks[WHITE][SQ_B1] - a2, c2
                                                        // 00000000 00000000 00000000 00000000 00000000 00000000 00000101 00000000

                                                        // PawnAttacks[BLACK][SQ_B7] - a6, c6
                                                        // 00000000 00000000 00000101 00000000 00000000 00000000 00000000 00000000

//---------------------------------------------------------------------------------------------------------------------------------
// Magic
//
// Magic boards are used to find what squares a sliding piece can attack, using occupancy Bitboard.
//
//  .mask   (Bitboard)      : Squares that a sliding piece can move/attack.
//                          (The mask does not include edges, b/c if a Rook can move to h7, can always move/attack to h8 on h file.
//                          This idea reduces the size of the table)
//  .shift   (unsigned int)  : (For 64 bit) 64 - (The number of non-zero bits in the mask)

//  .magic  (Bitboard)
//  attacks[index]
//  (occupied & mask) * magic >> shift = index
//                          : Occupancy is all possible piece combinations on the mask. Since we don't use the edges,
//                            There are 6+6 squares which makes 2^(6+6)=4096 possible combinations.
//

    // Mask - squares that a piece can attack, except edge squares. 
    // RookMagics[SQ_A1].mask  = a2-a7, b1-g1
    //                          00000000 00000001 00000001 00000001 00000001 00000001 00000001 01111110
    // RookMagics[SQ_C1].mask  = b1, d1-g1, c2-c7
    //                          00000000 00000100 00000100 00000100 00000100 00000100 00000100 01111010


// Example:
// index = ((Occupancy & mask) * magic) >> shift
// attacks[index]

// Rook A1 
// Magic :	756605012284543520

// Occupancy & mask: 2	(there is a piece on b1)
//
// Multiple occupancy with magic number
// (O & M) * magic = 2 * 756605012284543520  = 1513210024569087040
// 
// shift 52 bits to the right, and find index = 336
//
// Use: RookMagics[SQ_A1].attacks[336]

// there are two pieces on b1 and c1
// Occupancy & mask: 6
// 6  * magic = 4539630073707261120
// >> shift 52
// index: 1008

// there are three pieces on b1, c1 and d1
// Occupancy & mask: 14
// 14 * magic = 10592470171983609280
// >> shift 52
// index: 2352



/// Magic holds all magic bitboards relevant data for a single square
struct Magic {
  Bitboard  mask;
  Bitboard  magic;
  Bitboard* attacks;
  unsigned  shift;

  // Compute the attack's index using the 'magic bitboards' approach
  unsigned index(Bitboard occupied) const {

    if (HasPext)
        return unsigned(pext(occupied, mask));

    if (Is64Bit)
        return unsigned(((occupied & mask) * magic) >> shift);

    unsigned lo = unsigned(occupied) & unsigned(mask);
    unsigned hi = unsigned(occupied >> 32) & unsigned(mask >> 32);
    return (lo * unsigned(magic) ^ hi * unsigned(magic >> 32)) >> shift;
  }
};

extern Magic RookMagics[SQUARE_NB];
extern Magic BishopMagics[SQUARE_NB];


/// Overloads of bitwise operators between a Bitboard and a Square for testing
/// whether a given bit is set in a bitboard, and for setting and clearing bits.

inline Bitboard operator&(Bitboard b, Square s) {
  assert(s >= SQ_A1 && s <= SQ_H8);
  return b & SquareBB[s];
}

inline Bitboard operator|(Bitboard b, Square s) {
  assert(s >= SQ_A1 && s <= SQ_H8);
  return b | SquareBB[s];
}

inline Bitboard operator^(Bitboard b, Square s) {
  assert(s >= SQ_A1 && s <= SQ_H8);
  return b ^ SquareBB[s];
}

inline Bitboard& operator|=(Bitboard& b, Square s) {
  assert(s >= SQ_A1 && s <= SQ_H8);
  return b |= SquareBB[s];
}

inline Bitboard& operator^=(Bitboard& b, Square s) {
  assert(s >= SQ_A1 && s <= SQ_H8);
  return b ^= SquareBB[s];
}

constexpr bool more_than_one(Bitboard b) {
  return b & (b - 1);
}

/// rank_bb() and file_bb() return a bitboard representing all the squares on
/// the given file or rank.

inline Bitboard rank_bb(Rank r) {
  return RankBB[r];
}

inline Bitboard rank_bb(Square s) {
  return RankBB[rank_of(s)];
}

inline Bitboard file_bb(File f) {
  return FileBB[f];
}

inline Bitboard file_bb(Square s) {
  return FileBB[file_of(s)];
}


/// shift() moves a bitboard one step along direction D (mainly for pawns)

template<Direction D>
constexpr Bitboard shift(Bitboard b) {
  return  D == NORTH      ?  b             << 8 : D == SOUTH      ?  b             >> 8
        : D == EAST       ? (b & ~FileHBB) << 1 : D == WEST       ? (b & ~FileABB) >> 1
        : D == NORTH_EAST ? (b & ~FileHBB) << 9 : D == NORTH_WEST ? (b & ~FileABB) << 7
        : D == SOUTH_EAST ? (b & ~FileHBB) >> 7 : D == SOUTH_WEST ? (b & ~FileABB) >> 9
        : 0;
}


/// pawn_attacks_bb() returns the pawn attacks for the given color from the
/// squares in the given bitboard.

template<Color C>
constexpr Bitboard pawn_attacks_bb(Bitboard b) {
  return C == WHITE ? shift<NORTH_WEST>(b) | shift<NORTH_EAST>(b)
                    : shift<SOUTH_WEST>(b) | shift<SOUTH_EAST>(b);
}


/// adjacent_files_bb() returns a bitboard representing all the squares on the
/// adjacent files of the given one.

inline Bitboard adjacent_files_bb(File f) {
  return AdjacentFilesBB[f];
}


/// between_bb() returns a bitboard representing all the squares between the two
/// given ones. For instance, between_bb(SQ_C4, SQ_F7) returns a bitboard with
/// the bits for square d5 and e6 set. If s1 and s2 are not on the same rank, file
/// or diagonal, 0 is returned.

inline Bitboard between_bb(Square s1, Square s2) {
  return BetweenBB[s1][s2];
}


/// forward_ranks_bb() returns a bitboard representing the squares on all the ranks
/// in front of the given one, from the point of view of the given color. For instance,
/// forward_ranks_bb(BLACK, SQ_D3) will return the 16 squares on ranks 1 and 2.

inline Bitboard forward_ranks_bb(Color c, Square s) {
  return ForwardRanksBB[c][rank_of(s)];
}


/// forward_file_bb() returns a bitboard representing all the squares along the line
/// in front of the given one, from the point of view of the given color:
///      ForwardFileBB[c][s] = forward_ranks_bb(c, s) & file_bb(s)

inline Bitboard forward_file_bb(Color c, Square s) {
  return ForwardFileBB[c][s];
}


/// pawn_attack_span() returns a bitboard representing all the squares that can be
/// attacked by a pawn of the given color when it moves along its file, starting
/// from the given square:
///      PawnAttackSpan[c][s] = forward_ranks_bb(c, s) & adjacent_files_bb(file_of(s));

inline Bitboard pawn_attack_span(Color c, Square s) {
  return PawnAttackSpan[c][s];
}


/// passed_pawn_mask() returns a bitboard mask which can be used to test if a
/// pawn of the given color and on the given square is a passed pawn:
///      PassedPawnMask[c][s] = pawn_attack_span(c, s) | forward_file_bb(c, s)

inline Bitboard passed_pawn_mask(Color c, Square s) {
  return PassedPawnMask[c][s];
}


/// aligned() returns true if the squares s1, s2 and s3 are aligned either on a
/// straight or on a diagonal line.

inline bool aligned(Square s1, Square s2, Square s3) {
  return LineBB[s1][s2] & s3;
}


/// distance() functions return the distance between x and y, defined as the
/// number of steps for a king in x to reach y. Works with squares, ranks, files.

template<typename T> inline int distance(T x, T y) { return x < y ? y - x : x - y; }
template<> inline int distance<Square>(Square x, Square y) { return SquareDistance[x][y]; }

template<typename T1, typename T2> inline int distance(T2 x, T2 y);
template<> inline int distance<File>(Square x, Square y) { return distance(file_of(x), file_of(y)); }
template<> inline int distance<Rank>(Square x, Square y) { return distance(rank_of(x), rank_of(y)); }


/// attacks_bb() returns a bitboard representing all the squares attacked by a
/// piece of type Pt (bishop or rook) placed on 's'.

template<PieceType Pt>
inline Bitboard attacks_bb(Square s, Bitboard occupied) {

  const Magic& m = Pt == ROOK ? RookMagics[s] : BishopMagics[s];
  return m.attacks[m.index(occupied)];
}

inline Bitboard attacks_bb(PieceType pt, Square s, Bitboard occupied) {

  assert(pt != PAWN);

  switch (pt)
  {
  case BISHOP: return attacks_bb<BISHOP>(s, occupied);
  case ROOK  : return attacks_bb<  ROOK>(s, occupied);
  case QUEEN : return attacks_bb<BISHOP>(s, occupied) | attacks_bb<ROOK>(s, occupied);
  default    : return PseudoAttacks[pt][s];
  }
}


/// popcount() counts the number of non-zero bits in a bitboard

inline int popcount(Bitboard b) {

#ifndef USE_POPCNT

  extern uint8_t PopCnt16[1 << 16];
  union { Bitboard bb; uint16_t u[4]; } v = { b };
  return PopCnt16[v.u[0]] + PopCnt16[v.u[1]] + PopCnt16[v.u[2]] + PopCnt16[v.u[3]];

#elif defined(_MSC_VER) || defined(__INTEL_COMPILER)

  return (int)_mm_popcnt_u64(b);

#else // Assumed gcc or compatible compiler

  return __builtin_popcountll(b);

#endif
}


/// lsb() and msb() return the least/most significant bit in a non-zero bitboard

#if defined(__GNUC__)  // GCC, Clang, ICC

inline Square lsb(Bitboard b) {
  assert(b);
  return Square(__builtin_ctzll(b));
}

inline Square msb(Bitboard b) {
  assert(b);
  return Square(63 ^ __builtin_clzll(b));
}

#elif defined(_MSC_VER)  // MSVC

#ifdef _WIN64  // MSVC, WIN64

inline Square lsb(Bitboard b) {
  assert(b);
  unsigned long idx;
  _BitScanForward64(&idx, b);
  return (Square) idx;
}

inline Square msb(Bitboard b) {
  assert(b);
  unsigned long idx;
  _BitScanReverse64(&idx, b);
  return (Square) idx;
}

#else  // MSVC, WIN32

inline Square lsb(Bitboard b) {
  assert(b);
  unsigned long idx;

  if (b & 0xffffffff) {
      _BitScanForward(&idx, int32_t(b));
      return Square(idx);
  } else {
      _BitScanForward(&idx, int32_t(b >> 32));
      return Square(idx + 32);
  }
}

inline Square msb(Bitboard b) {
  assert(b);
  unsigned long idx;

  if (b >> 32) {
      _BitScanReverse(&idx, int32_t(b >> 32));
      return Square(idx + 32);
  } else {
      _BitScanReverse(&idx, int32_t(b));
      return Square(idx);
  }
}

#endif

#else  // Compiler is neither GCC nor MSVC compatible

#error "Compiler not supported."

#endif


/// pop_lsb() finds and clears the least significant bit in a non-zero bitboard

inline Square pop_lsb(Bitboard* b) {
  const Square s = lsb(*b);
  *b &= *b - 1;
  return s;
}


/// frontmost_sq() and backmost_sq() return the square corresponding to the
/// most/least advanced bit relative to the given color.

inline Square frontmost_sq(Color c, Bitboard b) { return c == WHITE ? msb(b) : lsb(b); }
inline Square  backmost_sq(Color c, Bitboard b) { return c == WHITE ? lsb(b) : msb(b); }

#endif // #ifndef BITBOARD_H_INCLUDED
