# #import math
# #import wpilib
# #from wpilib import DoubleSolenoid

# #import constants

#import commands2
# import commands2.cmd


# class GripperSubsystem(commands2.SubsystemBase):

#     Game_Piece_None = 0
#     Game_Piece_Note = 1

#     def __init__(self):
#         super().__init__()

        
#         self.game_piece = self.Game_Piece_None

#     def setGamePiece(self, game_piece):
#         self.game_piece = game_piece

    

   
#     def picked_up_note_cmd(self):
#         return commands2.cmd.runOnce(
#             lambda: self.setGamePiece(self.Game_Piece_Note), []
#         )

#     def wait_for_game_piece_cmd(self):
#         return commands2.cmd.waitUntil(
#             lambda: self.game_piece != self.Game_Piece_None, []
#         )

#     def dropped_game_piece_cmd(self):
#         return commands2.cmd.runOnce(
#             lambda: self.setGamePiece(self.Game_Piece_None), []
#         )


