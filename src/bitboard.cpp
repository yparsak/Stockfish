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

#include <algorithm>

#include "bitboard.h"
#include "misc.h"

uint8_t PopCnt16[1 << 16];
int SquareDistance[SQUARE_NB][SQUARE_NB];

Bitboard SquareBB[SQUARE_NB];
Bitboard FileBB[FILE_NB];
Bitboard RankBB[RANK_NB];
Bitboard AdjacentFilesBB[FILE_NB];
Bitboard ForwardRanksBB[COLOR_NB][RANK_NB];
Bitboard BetweenBB[SQUARE_NB][SQUARE_NB];
Bitboard LineBB[SQUARE_NB][SQUARE_NB];
Bitboard DistanceRingBB[SQUARE_NB][8];
Bitboard ForwardFileBB[COLOR_NB][SQUARE_NB];
Bitboard PassedPawnMask[COLOR_NB][SQUARE_NB];
Bitboard PawnAttackSpan[COLOR_NB][SQUARE_NB];
Bitboard PseudoAttacks[PIECE_TYPE_NB][SQUARE_NB];
Bitboard PawnAttacks[COLOR_NB][SQUARE_NB];

Magic RookMagics[SQUARE_NB];
Magic BishopMagics[SQUARE_NB];

namespace {

  Bitboard RookTable[0x19000];  // To store rook attacks
  Bitboard BishopTable[0x1480]; // To store bishop attacks

  void init_magics(Bitboard table[], Magic magics[], Direction directions[]);

  // popcount16() counts the non-zero bits using SWAR-Popcount algorithm

  unsigned popcount16(unsigned u) {
    u -= (u >> 1) & 0x5555U;
    u = ((u >> 2) & 0x3333U) + (u & 0x3333U);
    u = ((u >> 4) + u) & 0x0F0FU;
    return (u * 0x0101U) >> 8;
  }
}

const std::string Bitboards::dump() {
    std::string s = "";

//    s += "All Squares  "+printBB(AllSquares);
//    s += "Dark Squares "+printBB(DarkSquares);
//    s += "FileABB      "+printBB(FileABB);
//    s += "FileBBB      "+printBB(FileBBB);
//    s += "FileCBB      "+printBB(FileCBB);
//    s += "FileDBB      "+printBB(FileDBB);
//    s += "FileEBB      "+printBB(FileEBB);
//    s += "FileFBB      "+printBB(FileFBB);
//    s += "FileGBB      "+printBB(FileGBB);
//    s += "FileHBB      "+printBB(FileHBB);
//    s += "Rank1BB      "+printBB(Rank1BB);
//    s += "Rank2BB      "+printBB(Rank2BB);
//    s += "Rank3BB      "+printBB(Rank3BB);
//    s += "Rank4BB      "+printBB(Rank4BB);
//    s += "Rank5BB      "+printBB(Rank5BB);
//    s += "Rank6BB      "+printBB(Rank6BB);
//    s += "Rank7BB      "+printBB(Rank7BB);
//    s += "Rank8BB      "+printBB(Rank8BB);

//    s += "SquareDistance[SQ_A1][SQ_A2] " + std::to_string(SquareDistance[SQ_A1][SQ_A2]) + "\n";
//    s += "SquareDistance[SQ_A1][SQ_B2] " + std::to_string(SquareDistance[SQ_A1][SQ_B2]) + "\n";
//    s += "SquareDistance[SQ_A1][SQ_C2] " + std::to_string(SquareDistance[SQ_A1][SQ_C2]) + "\n";
//    s += "SquareDistance[SQ_A1][SQ_D3] " + std::to_string(SquareDistance[SQ_A1][SQ_D3]) + "\n";
//    s += "SquareDistance[SQ_A1][SQ_H1] " + std::to_string(SquareDistance[SQ_A1][SQ_H1]) + "\n";
//    s += "SquareDistance[SQ_A1][SQ_H8] " + std::to_string(SquareDistance[SQ_A1][SQ_H8]) + "\n";

//    s += "SquareBB[SQ_A1] "+printBB(SquareBB[SQ_A1]);
//    s += "SquareBB[SQ_B1] "+printBB(SquareBB[SQ_B1]);
//    s += "FileBB[FILE_A] "+printBB(FileBB[FILE_A]);
//    s += "FileBB[FILE_B] "+printBB(FileBB[FILE_B]);
//    s += "RankBB[RANK_1]  "+printBB(RankBB[RANK_1]);
//    s += "RankBB[RANK_2]  "+printBB(RankBB[RANK_2]);
//    s += "AdjacentFilesBB[FILE_A] " +printBB(AdjacentFilesBB[FILE_A]);
//    s += "AdjacentFilesBB[FILE_B] " +printBB(AdjacentFilesBB[FILE_B]);
//    s += "ForwardRanksBB[WHITE][RANK_1]" +printBB(ForwardRanksBB[WHITE][RANK_1]);
//    s += "ForwardRanksBB[WHITE][RANK_2]" +printBB(ForwardRanksBB[WHITE][RANK_2]);
//    s += "ForwardRanksBB[BLACK][RANK_8]" +printBB(ForwardRanksBB[BLACK][RANK_8]);
//    s += "ForwardRanksBB[BLACK][RANK_7]" +printBB(ForwardRanksBB[BLACK][RANK_7]);
    
//    s += "BetweenBB[SQ_A1][SQ_H1] " + printBB(BetweenBB[SQ_A1][SQ_H1]) ;
//    s += "BetweenBB[SQ_A1][SQ_H8] " + printBB(BetweenBB[SQ_A1][SQ_H8]) ;
//    s += "BetweenBB[SQ_A1][SQ_A2] " + printBB(BetweenBB[SQ_A1][SQ_A2]) ;

//    s += "LineBB[SQ_A1][SQ_H1] " + printBB(LineBB[SQ_A1][SQ_H1]) ;
//    s += "LineBB[SQ_A1][SQ_H8] " + printBB(LineBB[SQ_A1][SQ_H8]) ;
//    s += "LineBB[SQ_A1][SQ_A2] " + printBB(LineBB[SQ_A1][SQ_A2]) ;
//    s += "LineBB[SQ_A1][SQ_C2] " + printBB(LineBB[SQ_A1][SQ_C2]) ;
    
//    s += "DistanceRingBB[SQ_A1][0] " + printBB(DistanceRingBB[SQ_A1][0]);
//    s += "DistanceRingBB[SQ_A1][1] " + printBB(DistanceRingBB[SQ_A1][1]);
//    s += "DistanceRingBB[SQ_A1][2] " + printBB(DistanceRingBB[SQ_A1][2]);
    
//    s += "ForwardFileBB[WHITE][SQ_A1]" +printBB(ForwardFileBB[WHITE][SQ_A1]);
//    s += "ForwardFileBB[WHITE][SQ_A2]" +printBB(ForwardFileBB[WHITE][SQ_A2]);
//    s += "ForwardFileBB[BLACK][SQ_H8]" +printBB(ForwardFileBB[BLACK][SQ_H8]);

//    s += "PassedPawnMask[WHITE][SQ_B1]" +printBB(PassedPawnMask[WHITE][SQ_B1]);
//    s += "PassedPawnMask[WHITE][SQ_B2]" +printBB(PassedPawnMask[WHITE][SQ_B2]);
//    s += "PassedPawnMask[BLACK][SQ_B2]" +printBB(PassedPawnMask[BLACK][SQ_B2]);

//    s += "PawnAttackSpan[WHITE][SQ_B1]" +printBB(PawnAttackSpan[WHITE][SQ_B1]);
//    s += "PawnAttackSpan[WHITE][SQ_B2]" +printBB(PawnAttackSpan[WHITE][SQ_B2]);
//    s += "PawnAttackSpan[BLACK][SQ_B2]" +printBB(PawnAttackSpan[BLACK][SQ_B2]);

//    s += "PseudoAttacks[BISHOP][SQ_A1]" +printBB(PseudoAttacks[BISHOP][SQ_A1]);
//    s += "PseudoAttacks[ROOK][SQ_A1]"   +printBB(PseudoAttacks[ROOK][SQ_A1]);
//    s += "PseudoAttacks[QUEEN][SQ_A1]"  +printBB(PseudoAttacks[QUEEN][SQ_A1]);
//    s += "PseudoAttacks[KING][SQ_A1]"   +printBB(PseudoAttacks[KING][SQ_A1]);
//    s += "PseudoAttacks[KNIGHT][SQ_A1]" +printBB(PseudoAttacks[KNIGHT][SQ_A1]);

//    s += "PawnAttacks[WHITE][SQ_B1]" +printBB(PawnAttacks[WHITE][SQ_B1]);
//    s += "PawnAttacks[WHITE][SQ_B2]" +printBB(PawnAttacks[WHITE][SQ_B2]);
//    s += "PawnAttacks[BLACK][SQ_B7]" +printBB(PawnAttacks[BLACK][SQ_B7]);
    
//    Square sq = SQ_A1;
//    Bitboard edges = ((Rank1BB | Rank8BB) & ~rank_bb(sq)) | ((FileABB | FileHBB) & ~file_bb(sq));
//    s += pretty(edges);
//    Magic& m = RookMagics[sq];
//    s += pretty(m.mask);                // A1 b1-g1, a2-a7
//    s += std::to_string(m.shift)+"\n";  // 52
//    Bitboard occupancy[4096], b;
//    int size = 0;
//    b = size =0;
//    do {
//        occupancy[size] = b;
//        s += std::to_string(size) + " " + printBB(b);
//        size++;
//        b = (b - m.mask) & m.mask;
//    } while (b);

    
//    //s += "Rook Magics A1 " + pretty(RookMagics[SQ_A1].mask);
//    s += "Rook Magics C1 mask " + pretty(RookMagics[SQ_C1].mask);
    
//    for (Square sqr = SQ_A1; sqr <= SQ_H8; ++sqr) {
//        s +=    std::to_string(sqr)+"\t 64 - " +
//                std::to_string(popcount(RookMagics[sqr].mask)) + " = " +
//                std::to_string(RookMagics[sqr].shift) +"\n";
//    }
    
//    for (Square sq = SQ_A1; sq <= SQ_H8; ++sq)
//    {
//      s += "mask: "+pretty(RookMagics[sq].mask) +"\n";
//    }

//    for (Square sq = SQ_A1; sq <= SQ_H8; ++sq)
//    {
//      s += "magic: "+printBB(RookMagics[sq].magic) +"\n";
//    }

//    for (Square sq = SQ_A1; sq <= SQ_C1; ++sq)
//    {
//      s += printBB(sq);
//    }
 

/*
    Magic m = RookMagics[SQ_A1];
    Bitboard ocp = SQ_B1 << 1;
    s += "Rook A1 Magic "+std::to_string(m.magic)+"\n";
    for (Square sq = SQ_A1; sq <= SQ_D1; ++sq)
    {
      ocp = ocp | sq;
      s += "Mask "+printBB(ocp & m.mask) +"\n";
      s += "Mask "+std::to_string(ocp & m.mask) +"\n";
      Bitboard b = (ocp & m.mask) * m.magic;
      s += "O&M * Magic "+ std::to_string(b) +"\n";
      s += "shift "+ std::to_string(m.shift) +"\n"; 
      b = b >> m.shift;
      s += "Index "+ std::to_string(b) +"\n";
    }
*/
    return s;
}


const std::string Bitboards::printBB(Bitboard b) {
    std::string s = "";
    for (Rank r = RANK_8; r >= RANK_1; --r)
    {
        for (File f = FILE_H; f >= FILE_A; --f) {
            s += b & make_square(f,r) ? "1" : "0";
        }
        s+=" ";
    }
    s += "\n";
    return s;
}

/// Bitboards::pretty() returns an ASCII representation of a bitboard suitable
/// to be printed to standard output. Useful for debugging.

const std::string Bitboards::pretty(Bitboard b) {

  std::string s = "+---+---+---+---+---+---+---+---+\n";

  for (Rank r = RANK_8; r >= RANK_1; --r)
  {
      for (File f = FILE_A; f <= FILE_H; ++f)
          s += b & make_square(f, r) ? "| X " : "|   ";

      s += "|\n+---+---+---+---+---+---+---+---+\n";
  }

  return s;
}


/// Bitboards::init() initializes various bitboard tables. It is called at
/// startup and relies on global objects to be already zero-initialized.

void Bitboards::init() {

  for (unsigned i = 0; i < (1 << 16); ++i)
      PopCnt16[i] = (uint8_t) popcount16(i);

  for (Square s = SQ_A1; s <= SQ_H8; ++s)
      SquareBB[s] = (1ULL << s);

  for (File f = FILE_A; f <= FILE_H; ++f)
      FileBB[f] = f > FILE_A ? FileBB[f - 1] << 1 : FileABB;

  for (Rank r = RANK_1; r <= RANK_8; ++r)
      RankBB[r] = r > RANK_1 ? RankBB[r - 1] << 8 : Rank1BB;

  for (File f = FILE_A; f <= FILE_H; ++f)
      AdjacentFilesBB[f] = (f > FILE_A ? FileBB[f - 1] : 0) | (f < FILE_H ? FileBB[f + 1] : 0);

  for (Rank r = RANK_1; r < RANK_8; ++r)
      ForwardRanksBB[WHITE][r] = ~(ForwardRanksBB[BLACK][r + 1] = ForwardRanksBB[BLACK][r] | RankBB[r]);

  for (Color c = WHITE; c <= BLACK; ++c)
      for (Square s = SQ_A1; s <= SQ_H8; ++s)
      {
          ForwardFileBB [c][s] = ForwardRanksBB[c][rank_of(s)] & FileBB[file_of(s)];
          PawnAttackSpan[c][s] = ForwardRanksBB[c][rank_of(s)] & AdjacentFilesBB[file_of(s)];
          PassedPawnMask[c][s] = ForwardFileBB [c][s] | PawnAttackSpan[c][s];
      }

  for (Square s1 = SQ_A1; s1 <= SQ_H8; ++s1)
      for (Square s2 = SQ_A1; s2 <= SQ_H8; ++s2)
          if (s1 != s2)
          {
              SquareDistance[s1][s2] = std::max(distance<File>(s1, s2), distance<Rank>(s1, s2));
              DistanceRingBB[s1][SquareDistance[s1][s2]] |= s2;
          }

  int steps[][5] = { {}, { 7, 9 }, { 6, 10, 15, 17 }, {}, {}, {}, { 1, 7, 8, 9 } };

  for (Color c = WHITE; c <= BLACK; ++c)
      for (PieceType pt : { PAWN, KNIGHT, KING })
          for (Square s = SQ_A1; s <= SQ_H8; ++s)
              for (int i = 0; steps[pt][i]; ++i)
              {
                  Square to = s + Direction(c == WHITE ? steps[pt][i] : -steps[pt][i]);

                  if (is_ok(to) && distance(s, to) < 3)
                  {
                      if (pt == PAWN)
                          PawnAttacks[c][s] |= to;
                      else
                          PseudoAttacks[pt][s] |= to;
                  }
              }

  Direction RookDirections[] = { NORTH, EAST, SOUTH, WEST };
  Direction BishopDirections[] = { NORTH_EAST, SOUTH_EAST, SOUTH_WEST, NORTH_WEST };

  init_magics(RookTable, RookMagics, RookDirections);
  init_magics(BishopTable, BishopMagics, BishopDirections);

  for (Square s1 = SQ_A1; s1 <= SQ_H8; ++s1)
  {
      PseudoAttacks[QUEEN][s1]  = PseudoAttacks[BISHOP][s1] = attacks_bb<BISHOP>(s1, 0);
      PseudoAttacks[QUEEN][s1] |= PseudoAttacks[  ROOK][s1] = attacks_bb<  ROOK>(s1, 0);

      for (PieceType pt : { BISHOP, ROOK })
          for (Square s2 = SQ_A1; s2 <= SQ_H8; ++s2)
          {
              if (!(PseudoAttacks[pt][s1] & s2))
                  continue;

              LineBB[s1][s2] = (attacks_bb(pt, s1, 0) & attacks_bb(pt, s2, 0)) | s1 | s2;
              BetweenBB[s1][s2] = attacks_bb(pt, s1, SquareBB[s2]) & attacks_bb(pt, s2, SquareBB[s1]);
          }
  }
}


namespace {

  Bitboard sliding_attack(Direction directions[], Square sq, Bitboard occupied) {

    Bitboard attack = 0;

    for (int i = 0; i < 4; ++i)
        for (Square s = sq + directions[i];
             is_ok(s) && distance(s, s - directions[i]) == 1;
             s += directions[i])
        {
            attack |= s;

            if (occupied & s)
                break;
        }

    return attack;
  }


  // init_magics() computes all rook and bishop attacks at startup. Magic
  // bitboards are used to look up attacks of sliding pieces. As a reference see
  // chessprogramming.wikispaces.com/Magic+Bitboards. In particular, here we
  // use the so called "fancy" approach.

  void init_magics(Bitboard table[], Magic magics[], Direction directions[]) {

    // Optimal PRNG seeds to pick the correct magics in the shortest time
    int seeds[][RANK_NB] = { { 8977, 44560, 54343, 38998,  5731, 95205, 104912, 17020 },
                             {  728, 10316, 55013, 32803, 12281, 15100,  16645,   255 } };

    // Since we don't use edges, it will be 2^(6+6) = 4096
    Bitboard occupancy[4096], reference[4096], edges, b;
    int epoch[4096] = {}, cnt = 0, size = 0;

    for (Square s = SQ_A1; s <= SQ_H8; ++s)
    {
        // Board edges are not considered in the relevant occupancies
        edges = ((Rank1BB | Rank8BB) & ~rank_bb(s)) | ((FileABB | FileHBB) & ~file_bb(s));
        
        // Given a square 's', the mask is the bitboard of sliding attacks from
        // 's' computed on an empty board. The index must be big enough to contain
        // all the attacks for each possible subset of the mask and so is 2 power
        // the number of 1s of the mask. Hence we deduce the size of the shift to
        // apply to the 64 or 32 bits word to get the index.
        Magic& m = magics[s];
        m.mask  = sliding_attack(directions, s, 0) & ~edges;
        m.shift = (Is64Bit ? 64 : 32) - popcount(m.mask);

        // Set the offset for the attacks table of the square. We have individual
        // table sizes for each square with "Fancy Magic Bitboards".
        m.attacks = s == SQ_A1 ? table : magics[s - 1].attacks + size;

        // Use Carry-Rippler trick to enumerate all subsets of masks[s] and
        // store the corresponding sliding attack bitboard in reference[].
        
        // Loop from 0 to 4095. There are 2^(6+6)=4096 possible occupancy combinations.
        b = size = 0;
        do {
            occupancy[size] = b;
            reference[size] = sliding_attack(directions, s, b);
            
            if (HasPext)
                m.attacks[pext(b, m.mask)] = reference[size];

            size++;
            b = (b - m.mask) & m.mask;  //increment b, using mask
        } while (b);

        // (if HasPext) then this is set above, so continue
        // m.attacks[pext(b, m.mask)] = reference[size];
        if (HasPext)
            continue;
        
        PRNG rng(seeds[Is64Bit][rank_of(s)]);

        // Find a magic for square 's' picking up an (almost) random number
        // until we find the one that passes the verification test.
        for (int i = 0; i < size; )
        {
            for (m.magic = 0; popcount((m.magic * m.mask) >> 56) < 6; )
                m.magic = rng.sparse_rand<Bitboard>();

            // A good magic must map every possible occupancy to an index that
            // looks up the correct sliding attack in the attacks[s] database.
            // Note that we build up the database for square 's' as a side
            // effect of verifying the magic. Keep track of the attempt count
            // and save it in epoch[], little speed-up trick to avoid resetting
            // m.attacks[] after every failed attempt.
            for (++cnt, i = 0; i < size; ++i)
            {
                unsigned idx = m.index(occupancy[i]);

                if (epoch[idx] < cnt)
                {
                    epoch[idx] = cnt;
                    m.attacks[idx] = reference[i];
                }
                else if (m.attacks[idx] != reference[i])
                    break;
            }
        }
    }
  }
}
